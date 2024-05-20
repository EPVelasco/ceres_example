// Este programa es un nodo de ROS que se susceibe a un mensaje tipo Pose Arrya y convierte toda la informacion a un tipo PCL.
// esto para poder utulizar las funciones de PCL de filtrado ( Aun por ver si funciona). debido a que las poses se soncforman por informacion
// de xyz y un quaternion, lso datos son generados en PCL con un tipo de objeto OrbPointPCL. Este tipo de punto contiene toda la informacion
// de pose array en un objeto de la libreria PCL.
// Luego se publica esta nube de puntos para verificar si ha sido filtrada. Se prentende realizar un filtrado de la nube mediante el voxelizado
// o por caractersiticas del ORB como su score de harris o por medio de criterios del usuario , aún por analizar.

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>

#include <nav_msgs/Odometry.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>


///// para sincronizar los topics
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


#include <iostream>
#include <fstream>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <chrono>



///////////////////////////////////////////////// FUNCIONES de CERES/////////////////////////////////////////////////////////

// funcion quaterniones left and rigth
template<typename T>
Eigen::Matrix<T, 4, 4> quaternion_left(const Eigen::Quaternion<T>& q) {
    Eigen::Matrix<T, 4, 4> Q1;

    Q1 << q.w(), -q.x(), -q.y(), -q.z(),
          q.x(),  q.w(), -q.z(),  q.y(),
          q.y(),  q.z(),  q.w(), -q.x(),
          q.z(), -q.y(),  q.x(),  q.w();

    return Q1;
}

template<typename T>
Eigen::Matrix<T, 4, 4> quaternion_right(const Eigen::Quaternion<T>& q) {
    Eigen::Matrix<T, 4, 4> Q1;

    Q1 << q.w(), -q.x(), -q.y(), -q.z(),
          q.x(),  q.w(),  q.z(), -q.y(),
          q.y(), -q.z(),  q.w(),  q.x(),
          q.z(),  q.y(), -q.x(),  q.w();

    return Q1;
}

template<typename T>
Eigen::Quaternion<T> conjugate_quaternion(const Eigen::Quaternion<T>& q) {
    return Eigen::Quaternion<T>(q.w(), -q.x(), -q.y(), -q.z());
}


struct Point {
    double x, y, z;
};

struct TransformationCostFunction {
    TransformationCostFunction(const Eigen::Vector3d& source_, const Eigen::Vector3d& dest_)
        : source(source_), dest(dest_) {}

    template <typename T>
    bool operator()(const T* const quater, const T* const translation, T* residuals) const {
        Eigen::Quaternion<T> eigen_quaternion(quater[0], quater[1], quater[2], quater[3]);
        Eigen::Matrix<T, 4, 1> eigen_translation(T(0.0), translation[0], translation[1], translation[2]);

        Eigen::Matrix<T, 4, 1> source_T(T(0.0), T(source[0]),T(source[1]),T(source[2]));

        Eigen::Matrix<T, 4, 1> transformed_source = quaternion_left(eigen_quaternion)* 
                                                    quaternion_right(conjugate_quaternion(eigen_quaternion))* 
                                                    source_T + eigen_translation;

        residuals[0] = transformed_source[1] - dest[0];
        residuals[1] = transformed_source[2] - dest[1];
        residuals[2] = transformed_source[3] - dest[2];

        return true;
    }

private:
    const Eigen::Vector3d source;
    const Eigen::Vector3d dest;
};


// Definición del tipo de punto personalizado OrbPointPCL
struct OrbPointPCL
{
    PCL_ADD_POINT4D;
    float quaternion[4];
    int match; 
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

// Registro de campos para PCL
POINT_CLOUD_REGISTER_POINT_STRUCT(OrbPointPCL,
                                  (float, x, x)(float, y, y)(float, z, z)
                                  (float, quaternion[0], quat0)(float, quaternion[1], quat1)
                                  (float, quaternion[2], quat2)(float, quaternion[3], quat3)
                                  (int, match, match)) 

// Definición del tipo de nube de puntos
typedef pcl::PointCloud<OrbPointPCL> PointORB;
typedef pcl::PointCloud<pcl::PointXYZ> PointPCL;
typedef std::chrono::high_resolution_clock Clock;

PointORB::Ptr pcl_cloud(new PointORB);
ros::Publisher pointcloud_pub;
ros::Publisher  poseMap_pub;

geometry_msgs::PoseArray pose_map_msg;
/////// mapas decalracion
PointORB::Ptr OrbMap(new PointORB);
PointPCL::Ptr PclMap(new PointPCL);

//// Kdtree decalracion:
pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtreeMap;


bool is_odom_inited = false;

int cont_frames = 0; // variable para contar cunatos frames han pasado, de esta manera puedo verificar cada 30 frames cuales 
                     // puntos llegaron a un match de 0.
int limit_match =50; // numero maximo de match que puede acumular cada punto



////////////77777 Odoemtria

Eigen::Isometry3d    odom_prev = Eigen::Isometry3d::Identity();
Eigen::Isometry3d    odom_prev_ceres = Eigen::Isometry3d::Identity();
Eigen::Isometry3d    last_odom = Eigen::Isometry3d::Identity();


//////////////////////////////////////////////////// Resultados del ceres se guardan en

Eigen::Quaterniond quater = Eigen::Quaterniond::Identity();  // Inicialización del cuaternión
Eigen::Vector3d translation = Eigen::Vector3d::Zero();


/////////////////////////// transformaciones del sistema en cada instante de tiempo

Eigen::Quaterniond q_w_curr(1,0,0,0);
Eigen::Vector3d t_w_curr(0,0,0);

Eigen::Quaterniond q_prev(1,0,0,0);
Eigen::Vector3d t_prev(0,0,0);


//////////////////////// Decalracion de las funciones que tengo en el programa /////////////////////////

void initMap(PointORB::Ptr curr_features);
PointPCL::Ptr orbCloudToXYZ(const PointORB::Ptr& orbCloud);
void addCostFactor(const PointORB::Ptr& inputOrb, const PointORB::Ptr& mapOrb);
void pointAssociateToMap(pcl::PointXYZ const *const pi, pcl::PointXYZ *const po);
void updateMap (const PointORB::Ptr& curr_features);
void filterMap();
////////////////////////////////////////////////////////////////////////////////////////////////////////

//void poseArrayCallback(const geometry_msgs::PoseArrayConstPtr &pose_array_msg)
void callback(const nav_msgs::OdometryConstPtr& odom_msg, const geometry_msgs::PoseArrayConstPtr& pose_array_msg) 
{
    auto t1 = Clock::now();	

    // contador de frames:
    cont_frames++;

  

    // Limpiar la nube de puntos existente
    pcl_cloud->clear();

    // Iterar sobre los mensajes de poses
    for (const auto &pose : pose_array_msg->poses)
    {
        OrbPointPCL point;
        point.x = pose.position.x;
        point.y = pose.position.y;
        point.z = pose.position.z;
        point.quaternion[0] = pose.orientation.w; // w
        point.quaternion[1] = pose.orientation.x; // x
        point.quaternion[2] = pose.orientation.y; // y
        point.quaternion[3] = pose.orientation.z; // z
      
        point.match = 0; // match del punto la nube actual tiene 0 ( este valor es para contar los match en el mapa local)

        pcl_cloud->push_back(point);
    }

    // Publicar la nube de puntos si tiene datos
    if (pcl_cloud->size() > 0)
    {
        if (!is_odom_inited)
        {
            initMap(pcl_cloud);
            is_odom_inited = true;
            ROS_INFO("Map initialized");
            kdtreeMap = pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZ>());
        }
        else
        {

            // Tomo los datos de odometria para mover mis puntos ( esto estoy haciendo con un optmizador distinto para ver como
            //va a quedar el mapa)

            t_w_curr << odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z;

            // Obtener la orientación del mensaje Odometry
            q_w_curr.w() = odom_msg->pose.pose.orientation.w;
            q_w_curr.x() = odom_msg->pose.pose.orientation.x;
            q_w_curr.y() = odom_msg->pose.pose.orientation.y;
            q_w_curr.z() = odom_msg->pose.pose.orientation.z;

            
        }


        // mapa con Kdtree
        kdtreeMap->setInputCloud(PclMap);
        //PointPCL::Ptr orbpoint2pcl = orbCloudToXYZ(pcl_cloud);
        addCostFactor(pcl_cloud,OrbMap);   /// funcion de costo envio el punto actual y el mapa en ORB, dentro de la funcion converito a pcl


        q_prev = q_w_curr;
        t_prev = t_w_curr;

        // Publicar la nube de puntos en el topic /orbPoints
        sensor_msgs::PointCloud2 pcl_msg;
        pcl::toROSMsg(*OrbMap, pcl_msg); // Publico la nube de puntos que queiro ver
        pcl_msg.header = pose_array_msg->header;
        pointcloud_pub.publish(pcl_msg);


        // publish orb map con orientaciones
        pose_map_msg.header.stamp = pose_array_msg->header.stamp;//ros::Time::now();
        pose_map_msg.header.frame_id = "os_sensor";  // Ajusta el marco de referencia 

        for (size_t i = 0; i < OrbMap->size(); ++i)
        {
            const OrbPointPCL& orbPoint = OrbMap->points[i];
            
            Eigen::Vector3d point_currORB(orbPoint.x, orbPoint.y, orbPoint.z);
            Eigen::Vector3d point_w = q_prev * point_currORB + t_prev;

            geometry_msgs::Pose poseORBmap; 

            // position
            poseORBmap.position.x = point_w.x();
            poseORBmap.position.y = point_w.y();
            poseORBmap.position.z = point_w.z();
            
            // oreintation
            poseORBmap.orientation.w = orbPoint.quaternion[0];
            poseORBmap.orientation.x = orbPoint.quaternion[1];
            poseORBmap.orientation.y = orbPoint.quaternion[2];
            poseORBmap.orientation.z = orbPoint.quaternion[3];            
            // add pose to the array
            pose_map_msg.poses.push_back(poseORBmap);

        }

    poseMap_pub.publish(pose_map_msg);
    pose_map_msg.poses.clear();

    auto t2= Clock::now();
    float time_t = std::chrono::duration_cast<std::chrono::nanoseconds>(t2-t1).count()/1000000.0;
	std::cout<<"Time_per_frame (ms): "<<time_t<<std::endl;
    }
}

void initMap(PointORB::Ptr curr_features)
{
    // Limpia los mapas existentes Esto se hace al principio del programa simpre para inicializar el mapa.
    OrbMap->clear(); 
    PclMap->clear();

    OrbMap->insert(OrbMap->end(), curr_features->begin(), curr_features->end());
    PclMap = orbCloudToXYZ(OrbMap);

}

// Función para convertir una nube de puntos OrbPointPCL a PointPCL
PointPCL::Ptr orbCloudToXYZ(const PointORB::Ptr& orbCloud)
{
    PointPCL::Ptr xyzCloud(new PointPCL);

    // Reserva el espacio necesario para la nube de puntos XYZ
    xyzCloud->resize(orbCloud->size());

    // Convierte cada punto de OrbPointPCL a pcl::PointXYZ
    for (size_t i = 0; i < orbCloud->size(); ++i)
    {
        const OrbPointPCL& orbPoint = orbCloud->points[i];
        pcl::PointXYZ& xyzPoint = xyzCloud->points[i];

        xyzPoint.x = orbPoint.x;
        xyzPoint.y = orbPoint.y;
        xyzPoint.z = orbPoint.z;
    }

    return xyzCloud;
}

//void OdomEstimationClass::addEdgeCostFactor(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc_in, const pcl::PointCloud<pcl::PointXYZ>::Ptr& map_in, ceres::Problem& problem, ceres::LossFunction *loss_function){


//// Esta funcion lo que hace es asociar los puntos de mi medicion actual (mi nube de puntos en orbpoint) con un mapa que tambien es Orb
// para esto lo que hago es asociar con kdtree pero solo puedo usar kdtree con los tipos de puntos de PCL (en un futuro se puede implemetar otro kdtree)
// el de ahora es sufiente para mi investigacion. 

void addCostFactor(const PointORB::Ptr& inputOrb, const PointORB::Ptr& mapOrb){

    PointPCL::Ptr inputPCL = orbCloudToXYZ(inputOrb);

    PointORB::Ptr aux_inputOrb = inputOrb;
    
    int num_match = 0; // numero de emparejamientos entre la nube de puntos actual y el mapa
    float dist = 1.0; // distancia maxima entre cada punto y sus vecinos cercanos
    int nearK = 5;   // numero de vecinos cercanos a analizar
    int k = 0;    // entero para guardar el indice del match, de esta manera este se le pone 1 punto al tener un match

    ceres::LossFunction *loss_function = new ceres::CauchyLoss(1.0);
    ceres::Problem::Options problem_options;
    ceres::Problem problem(problem_options);

    ////////////// Configurar el objeto ICP

    // pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    // icp.setInputSource(inputPCL);
    // icp.setInputTarget(PclMap);

    // // Ajustar los parámetros según sea necesario
    // icp.setMaxCorrespondenceDistance(1.0);  // Distancia máxima para considerar correspondencias
    // icp.setMaximumIterations(10);            // Número máximo de iteraciones del algoritmo
    // icp.setTransformationEpsilon(1e-8);      // Criterio de convergencia

    // // Realizar la alineación
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aligned(new pcl::PointCloud<pcl::PointXYZ>);
    // icp.align(*cloud_aligned);

    // Eigen::Matrix4d tf = icp.getFinalTransformation().cast<double>();

	// // Parse to affine to manage transforms
  	// Eigen::Affine3d tf_aff(Eigen::Affine3f::Identity());
  	// tf_aff.matrix() = tf;


    std::cout<<"Points: "<<std::endl;
    for (int i = 0; i < (int)inputPCL->points.size(); i++)
    {
        pcl::PointXYZ point_temp;
       
        OrbPointPCL aux_OrbPcl_temp = inputOrb->points[i];

        pointAssociateToMap(&(inputPCL->points[i]), &point_temp);

        // Eigen::Vector3d point_temp_eig = tf_aff.inverse() * Eigen::Vector3d(inputPCL->points[i].x,
        //                                                                     inputPCL->points[i].y,
        //                                                                     inputPCL->points[i].z);

        // point_temp.x = point_temp_eig.x();
        // point_temp.y = point_temp_eig.y();
        // point_temp.z = point_temp_eig.z();                                                                   


        //////////// quaternion del punto a asociar al mapa
        Eigen::Quaterniond OrbQuat(inputOrb->points[i].quaternion[0],
                                   inputOrb->points[i].quaternion[1],
                                   inputOrb->points[i].quaternion[2],
                                   inputOrb->points[i].quaternion[3]);

        /////////////////////////////////////////////////////

        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;       

        kdtreeMap->nearestKSearch(point_temp, nearK, pointSearchInd, pointSearchSqDis);

        // std::cout<<"XYZ point ***: "<<aux_OrbPcl_temp.x<<", "
        //                           <<aux_OrbPcl_temp.y<<", "
        //                           <<aux_OrbPcl_temp.z<<", "<<std::endl;

        // for (int ii = 0; ii < nearK; ii++)
        // {
        //     std::cout<<"distancia: "<<pointSearchSqDis[ii]<<std::endl;
        //     std::cout<<"XYZ map  : "<<mapOrb->points[pointSearchInd[ii]].x<<", "
        //                             <<mapOrb->points[pointSearchInd[ii]].y<<", "
        //                             <<mapOrb->points[pointSearchInd[ii]].z<<", "<<std::endl;

        // } 


        
        //std::vector<Eigen::Vector3d> nearFrames;
        Eigen::Vector3d center(0, 0, 0);

        double minAng = 2*M_PI; // angulo maximo entre 2 cauterniones. Este valor va a determinar cual es el frame que corresponde al otro
        int bestQuat = 0; // el mejor cuaternion. 
        int cont_kdtree = 0; //  verifico si hay kdtree en le rango de dsitancia para poder añadir en el optmizador
        for (int j = 0; j < nearK; j++)
        {
            if (pointSearchSqDis[j] > dist){

                for (int j = 0; j < nearK; j++)
                {
                    if(mapOrb->points[pointSearchInd[j]].match > 0 && mapOrb->points[pointSearchInd[j]].match < limit_match)
                        mapOrb->points[pointSearchInd[j]].match--;

                }
                continue;            
            }
            cont_kdtree++;

            // std::cout<<"punto kdtree : "<<mapOrb->points[pointSearchInd[j]].x<<", "
            //                             <<mapOrb->points[pointSearchInd[j]].y<<", "
            //                             <<mapOrb->points[pointSearchInd[j]].z<<", "<<std::endl;


            Eigen::Vector3d tmp(mapOrb->points[pointSearchInd[j]].x,
                                mapOrb->points[pointSearchInd[j]].y,
                                mapOrb->points[pointSearchInd[j]].z);

            Eigen::Quaterniond pointQuat(mapOrb->points[pointSearchInd[j]].quaternion[0],
                                            mapOrb->points[pointSearchInd[j]].quaternion[1],
                                            mapOrb->points[pointSearchInd[j]].quaternion[2],
                                            mapOrb->points[pointSearchInd[j]].quaternion[3]);


        //    std::cout<<"map"<< pointQuat.coeffs().transpose()<<std::endl;
            
            double distQ = OrbQuat.angularDistance(pointQuat);
            if (distQ <= minAng) { // aqui es para encontrar el mejor mach dependeindo de la diferencia angular entre el punto y el punto del map y tambien si no tuvo match previos. Esto apra evitar concidencias entre vartios puntos con el mismo frame
                minAng = distQ;
                bestQuat = pointSearchInd[j];
                k = j; // guardo el indice del mejor valor en un entrero para luego darle un punto      

            }
        }
        double max_dist_ang = 20 *M_PI/180.0;
        if (cont_kdtree == 0 || minAng>max_dist_ang) // condicion para que no entre al optimizador porque no encontre un kdtree adecuado o si la distancia angular entre caractersitcas ORB es muy grande 0,35 radianes aprox 20 grad
            continue;

        // aqui accedo a la variable match del punto para sumarle un punto se limita a 50, despues de esto no se suma

        if(mapOrb->points[pointSearchInd[k]].match < limit_match)
            mapOrb->points[pointSearchInd[k]].match++;

        /////////////////////////////////////////////////////////////////////////////////////////////////////////    
        //                           _____ ______ _____  ______  _____ 
        //                          / ____|  ____|  __ \|  ____|/ ____|
        //                         | |    | |__  | |__) | |__  | (___  
        //                         | |    |  __| |  _  /|  __|  \___ \ 
        //                         | |____| |____| | \ \| |____ ____) |
        //                          \_____|______|_|  \_\______|_____/ 
        

        // aqui vamos a enviar cada match para que el optimizador trabaje, vamos a encontrar el q t  ideal 


        
        
        Eigen::Vector3d   source_Point (point_temp.x,
                                        point_temp.y,
                                        point_temp.z);

        Eigen::Quaterniond source_Quat(inputOrb->points[i].quaternion[0],
                                            inputOrb->points[i].quaternion[1],
                                            inputOrb->points[i].quaternion[2],
                                            inputOrb->points[i].quaternion[3]);


        Eigen::Vector3d   bestMatchPoint(mapOrb->points[bestQuat].x,
                                            mapOrb->points[bestQuat].y,
                                            mapOrb->points[bestQuat].z);

        Eigen::Quaterniond bestMatchQuat(mapOrb->points[bestQuat].quaternion[0],
                                            mapOrb->points[bestQuat].quaternion[1],
                                            mapOrb->points[bestQuat].quaternion[2],
                                            mapOrb->points[bestQuat].quaternion[3]);

        //std::cout<<source_Point.transpose()<<", "<<bestMatchPoint.transpose()<<";"<<std::endl;

        

            Eigen::Vector3d p = source_Point; 
            Eigen::Vector3d q = bestMatchPoint;

            ceres::CostFunction* cost_function =
                new ceres::AutoDiffCostFunction<TransformationCostFunction, 3, 4, 3>(
                    new TransformationCostFunction(p, q));
            problem.AddResidualBlock(cost_function, loss_function, quater.coeffs().data(), translation.data());
        
        
        //////////////////////////////////////////////////////////Fin Ceres funcion de costo //////////////////////////////////77

        // Eliminar el punto en la posición i que es un match, esto para no poner un punto encima de otro y ahorrar memoria
        aux_inputOrb->points.erase(aux_inputOrb->points.begin() + i - num_match);
        num_match++;
        

    }

    // PArametrizando los resultados
    ceres::Manifold* quaternion_parameterization = new ceres::EigenQuaternionManifold;
    problem.AddParameterBlock(quater.coeffs().data(), 4, quaternion_parameterization);

    ///// REsultados de CERES 

    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    options.check_gradients = false;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 2000;
    options.gradient_check_relative_precision = 1e-6;
    //options.function_tolerance = 1e-6;
   // options.parameter_tolerance = 1e-6;



    // Resolver el problema de optimización
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << std::endl;


    // Mostrar resultados
    //std::cout << summary.BriefReport() << std::endl;
    std::cout << "Optimized Quaternion: " << quater.coeffs().transpose() << std::endl;
    std::cout << "Optimized Translation: " << translation.transpose() << std::endl;


    Eigen::Quaterniond quat_aux(quater.x(),quater.y(),quater.z(),quater.w());
    Eigen::Isometry3d odom_curr = odom_prev * Eigen::Translation3d(translation) * quat_aux;

    // Extrae cuaternión y traslación de la odometría actual
    Eigen::Quaterniond q_c(odom_curr.rotation());
    Eigen::Vector3d t_c = odom_curr.translation();

    // Actualiza la odometría previa
    odom_prev = odom_curr;

    // Imprime el cuaternión y la traslación
    std::cout << "Quaternion: " << q_c.coeffs().transpose() << std::endl;
    std::cout << "Translation: " << t_c.transpose() << std::endl;

    std::cout << "tamanno de puntos entrada: " << (int)inputPCL->points.size() << std::endl;
    std::cout << "tamanno de puntos aux    : " << (int)aux_inputOrb->points.size() << std::endl;

    if ((int)aux_inputOrb->points.size()>0){
        updateMap(aux_inputOrb);
        aux_inputOrb->clear();
    }
    //std::cout<<"num_match: " << num_match<<std::endl;

    // for (int j = 0; j < (int)OrbMap->points.size(); j++)       
    // {    

    //     if (OrbMap->points[j].match != 0)
    //         std::cout<<"Mt: "<<OrbMap->points[j].match; 

    // }
    

}

//funcion para actualizar cada punot de la medida al mapa local.
void pointAssociateToMap(pcl::PointXYZ const *const pi, pcl::PointXYZ *const po)
{
    Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
    Eigen::Vector3d point_w = q_prev * point_curr + t_prev;
    po->x = point_w.x();
    po->y = point_w.y();
    po->z = point_w.z();

}


void updateMap (const PointORB::Ptr& curr_features) // esta funcion añade datos al mapa local para el kdtree. Debes tomar enceunta el cambio de PointOrb que estas haciendo
{

    //PointORB::Ptr filter_features = filterfeatures(curr_features);

    // borrado de mapa ( aqui vamos a tener el criterio de borrado)
    filterMap();

    for (int j = 0; j < (int)curr_features->points.size(); j++)       
    {       
        
        Eigen::Vector3d point_curr(curr_features->points[j].x, curr_features->points[j].y, curr_features->points[j].z);
        Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;      //  aqui transformo de mi lectura local al mundo del mapa se debe toamr enceunta que esta seria la transformacion que em da el opti
                                                                        // entonces se debe revisar la trasnformacion en duales.
        OrbPointPCL point_temp;

        point_temp.x =point_w.x();
        point_temp.y =point_w.y();
        point_temp.z =point_w.z();
        point_temp.quaternion[0] = curr_features->points[j].quaternion[0]; // w
        point_temp.quaternion[1] = curr_features->points[j].quaternion[1]; // x
        point_temp.quaternion[2] = curr_features->points[j].quaternion[2]; // y
        point_temp.quaternion[3] = curr_features->points[j].quaternion[3]; // z
        point_temp.match= curr_features->points[j].match; // match del punto en teoria aqui se esta poniendo 0 ya que el curr feature siempre tiene 0, este valor va aumentando en el mapa no en la medida actual. 
                                                          // Debes tomar enceunta que el numero match aumenta SOLO en el mapa, y esto se da cuando kdtree+angulo encontro una concidencia.
        
        //pointAssociateToMap(&curr_features->points[j], &point_temp);

        OrbMap->push_back(point_temp);

    }
    PclMap = orbCloudToXYZ(OrbMap);


}

void filterMap () // esta funcion es para el filtrado de datos del mapa que se va generando, para eto tenemos el sigueinte criterio:
                  // El mapa se va filtrando dependiendo ede la variable match, esta variabla aumetna cuando kdtree+angulo encontro una concidencia entre el mapa
                  // y la nube de puntos actual, cuando este valor llega a 50 ya no se sigue aumentando. Si el valor llega a  50 no se borra al filtrar, asumiendo que
                  // este punto es un punto FUERTE (criterio nuestro). Si por alguna razon el punto de kdtree+anguulo no tiene considencia en 30 muestras este se resta 1 del match
                  // hasta llegar a 0. si ya llega a 0 este punto se borra del mapa. 
{
    if (cont_frames == 10){
        cont_frames = 0;
    int num_zeros= 0;
    for (int j = 0; j < (int)OrbMap->points.size(); j++)       
    {    

        if (OrbMap->points[j].match != 0)
            continue;

        OrbMap->points.erase(OrbMap->points.begin() + j - num_zeros);
        num_zeros++; 

       
    }
    std::cout<<"se borraron: "<<num_zeros<<"puntos "; 
    }
    
}

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "pose_array_to_pointcloud");
//     ros::NodeHandle nh;
//     ros::Subscriber pose_array_sub = nh.subscribe("/pose_array_topic", 10, poseArrayCallback);
//     pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/orbPoints", 1);
//     ros::spin();
//     return 0;
// }

int main(int argc, char** argv) {
    ros::init(argc, argv, "pose_array_to_pointcloud");
    ros::NodeHandle nh;

    // Definir los suscriptores para los dos topics
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "/odom", 1);
    message_filters::Subscriber<geometry_msgs::PoseArray> pose_array_sub(nh, "/pose_curr_topic", 1);

    // Definir la política de sincronización (aproximada en el tiempo)
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, geometry_msgs::PoseArray> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), odom_sub, pose_array_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/orbPoints", 1);
    poseMap_pub = nh.advertise<geometry_msgs::PoseArray>("/mapORB", 1, true);
    
    ros::spin();

    return 0;
}