#include <iostream>
#include <fstream>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

///////////////////////////////////////////////// FUNCIONES QUE LUEGO VAN EN LA LIBRERIA/////////////////////////////////////////////////////////
// Funcion para leer los puntos del bunny ( el formato del archivo es .txt y deben tener los puntos en cada linea un punto xyz)

template<typename T>
Eigen::Matrix<T,4,1> quatMult_mT( const Eigen::Matrix<T,4,1>& q1, const Eigen::Matrix<T,4,1>& q2) {
    Eigen::Matrix<T,4,1> resultQ;


    resultQ(0) = q1(0) * q2(0) - q1(1) * q2(1) - q1(2) * q2(2) - q1(3) * q2(3);
    resultQ(1) = q1(0) * q2(1) + q1(1) * q2(0) + q1(2) * q2(3) - q1(3) * q2(2);
    resultQ(2) = q1(0) * q2(2) + q1(2) * q2(0) + q1(3) * q2(1) - q1(1) * q2(3);
    resultQ(3) = q1(0) * q2(3) + q1(3) * q2(0) + q1(1) * q2(2) - q1(2) * q2(1);

    return resultQ;
}

Eigen::MatrixXf leerArchivoPuntos(const std::string& nombreArchivo) {
    std::ifstream archivo(nombreArchivo);
    if (!archivo.is_open()) {
        std::cerr << "Error al abrir el archivo: " << nombreArchivo << std::endl;
        return Eigen::MatrixXf();
    }
    std::vector<std::vector<float>> puntos; // Vector de vectores para almacenar temporalmente las coordenadas
    float x, y, z;
    while (archivo >> x >> y >> z) {
        puntos.push_back({x, y, z});
    }
    archivo.close();
    // Convierte el vector de vectores a una matriz Eigen::MatrixXf
    Eigen::MatrixXf matrizPuntos(puntos.size(), 3);
    for (int i = 0; i < puntos.size(); ++i)
        matrizPuntos.row(i) << puntos[i][0], puntos[i][1], puntos[i][2];
    
    return matrizPuntos;
}


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

template<typename T>
Eigen::Quaternion<T> quatMult_T( Eigen::Quaternion<T> q1,  Eigen::Quaternion<T> q2) {
    Eigen::Quaternion<T> resultQ;


    resultQ.w() = q1.w() * q2.w() - q1.x() * q2.x() - q1.y() * q2.y() - q1.z() * q2.z();
    resultQ.x() = q1.w() * q2.x() + q1.x() * q2.w() + q1.y() * q2.z() - q1.z() * q2.y();
    resultQ.y() = q1.w() * q2.y() + q1.y() * q2.w() + q1.z() * q2.x() - q1.x() * q2.z();
    resultQ.z() = q1.w() * q2.z() + q1.z() * q2.w() + q1.x() * q2.y() - q1.y() * q2.x();

    return resultQ;
}

template<typename T>
Eigen::Matrix<T, 8, 1> ToDQ_T( Eigen::Quaternion<T> rotacion,  Eigen::Matrix<T, 3, 1> translation) {
    
    Eigen::Quaternion<T> r;
    r.w() = rotacion.w();
    r.x() = rotacion.x();
    r.y() = rotacion.y();
    r.z() = rotacion.z();
    
    Eigen::Quaternion<T> t;
    t.w() = T(0.0);
    t.x() = translation(0);
    t.y() = translation(1);
    t.z() = translation(2);


    Eigen::Quaternion<T> d_aux = quatMult_T(t,r);
    Eigen::Matrix<T, 4, 1> d(d_aux.w(),d_aux.x(),d_aux.y(),d_aux.z());
    d = d*T(0.5);

    Eigen::Matrix<T, 8, 1> res_udq;
    
    res_udq << r.w(),r.x(),r.y(),r.z(),d(0),d(1),d(2),d(3); 

    return res_udq;
}

template<typename T>
Eigen::Matrix<T, 8, 1> dq_conjugate(Eigen::Matrix<T, 8, 1> dq)
{
    Eigen::Matrix<T, 8, 1> res_conj;
    res_conj << T(dq(0)), T(-dq(1)), T(-dq(2)), T(-dq(3)), T(dq(4)), T(-dq(5)), T(-dq(6)), T(-dq(7));
    return res_conj;
}

template<typename T>
Eigen::Matrix<T,8,1> dualquatMult(const Eigen::Matrix<T,8,1>& q1, const Eigen::Matrix<T,8,1>& q2) {
    Eigen::Matrix<T,8,1> res_mul;
    res_mul << T(1.0),T(0.0),T(0.0),T(0.0),T(0.0),T(0.0),T(0.0),T(0.0);
    res_mul << T(q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3]),
               T(q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2]),
               T(q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1]),
               T(q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0]),
               T(q1[0] * q2[4] - q1[1] * q2[5] - q1[2] * q2[6] - q1[3] * q2[7] + q1[4] * q2[0] - q1[5] * q2[1] - q1[6] * q2[2] - q1[7] * q2[3]),
               T(q1[0] * q2[5] + q1[1] * q2[4] + q1[2] * q2[7] - q1[3] * q2[6] + q1[4] * q2[1] + q1[5] * q2[0] + q1[6] * q2[3] - q1[7] * q2[2]),
               T(q1[0] * q2[6] - q1[1] * q2[7] + q1[2] * q2[4] + q1[3] * q2[5] + q1[4] * q2[2] - q1[5] * q2[3] + q1[6] * q2[0] + q1[7] * q2[1]),
               T(q1[0] * q2[7] + q1[1] * q2[6] - q1[2] * q2[5] + q1[3] * q2[4] + q1[4] * q2[3] + q1[5] * q2[2] - q1[6] * q2[1] + q1[7] * q2[0]);
    
    return res_mul;
}

template<typename T>
Eigen::Matrix<T, 4, 1> get_translation_dual( Eigen::Matrix<T, 8, 1> udq) {
    
    Eigen::Quaternion<T> quat;
    quat.w() = udq(0);
    quat.x() = udq(1);
    quat.y() = udq(2);
    quat.z() = udq(3);


    Eigen::Quaternion<T> dual;
    dual.w() = udq(4);
    dual.x() = udq(5);
    dual.y() = udq(6);
    dual.z() = udq(7);

    Eigen::Quaternion<T> t_aux = quatMult_T(dual,conjugate_quaternion(quat));
    Eigen::Matrix<T, 4, 1> t(t_aux.w(),t_aux.x(),t_aux.y(),t_aux.z());

    t = t * T(2.0);

    Eigen::Matrix<T, 4, 1> res_traslation(T(0.0),t(1),t(2),t(3)); 
    
    return res_traslation;
}

template<typename T>
Eigen::Matrix<T, 8, 1> log_dq(Eigen::Matrix<T, 8, 1> dq)
{

    T squared_norm_delta =  dq(1)*dq(1) + dq(2)*dq(2) + dq(3)*dq(3);
    Eigen::Matrix<T,4,1> aux;
    Eigen::Matrix<T, 8, 1> res_log;
    Eigen::Matrix<T, 4, 1> q;
    Eigen::Matrix<T, 4, 1> d;

    if (squared_norm_delta > T(0.0)){

        T norm_delta = sqrt(squared_norm_delta);
        T theta = atan2(norm_delta,dq(0));
        aux << theta, dq(1)/norm_delta, dq(2)/norm_delta, dq(3)/norm_delta;

        Eigen::Matrix<T,4,1> aux_2;
        aux_2 << T(0.0), aux(0)*aux(1), aux(0)*aux(2),aux(0)*aux(3);
        q = T(0.5) * aux_2; //primary
       // d = T(0.5) * get_translation_dual(dq); //dual

    }
    else
    {
        //aux << T(0.0), T(0.0), T(0.0), T(1.0);
        q << T(0.0),dq(1),dq(2),dq(3);
      //  d << T(0.0),dq(5),dq(6),dq(7);
    }

    d = T(0.5) * get_translation_dual(dq); //dual

    res_log << q(0),q(1),q(2),q(3),d(0),d(1),d(2),d(3);

    return res_log;
}

template<typename T>
Eigen::Matrix<T, 8, 1> exp_dq(Eigen::Matrix<T, 8, 1> q)
{

    Eigen::Matrix<T, 4, 1> prim_exp;
    Eigen::Matrix<T, 8, 1> res_exp;
    res_exp << T(1.0),T(0.0),T(0.0),T(0.0),T(0.0),T(0.0),T(0.0),T(0.0);
 
    Eigen::Quaternion<T> q_d;
            q_d.w()=q(4);
            q_d.x()=q(5);
            q_d.y()=q(6);
            q_d.z()=q(7); 

   // Eigen::Matrix<T,4,1> prim_dq(q(0),q(1),q(2),q(3));

    T squared_norm_delta =  q(1)*q(1) + q(2)*q(2) + q(3)*q(3);
    Eigen::Quaternion<T> dual_exp;
    
    if(squared_norm_delta > T(1e-10)){

        T phi = sqrt(squared_norm_delta);
        prim_exp(0) = cos(phi);
        prim_exp(1) = (sin(phi)/phi) * q(1);
        prim_exp(2) = (sin(phi)/phi) * q(2);
        prim_exp(3) = (sin(phi)/phi) * q(3);

        Eigen::Quaternion<T> prim_exp_quat;      /// convierto de matriz de 4x1  a un objeto de quaternion
        prim_exp_quat.w()=prim_exp(0);
        prim_exp_quat.x()=prim_exp(1);
        prim_exp_quat.y()=prim_exp(2);
        prim_exp_quat.z()=prim_exp(3);
        dual_exp = quatMult_T(q_d,prim_exp_quat);

    }

    else{

        prim_exp << T(1.0) ,q(1) ,q(2) , q(3);
        dual_exp.w() = q(4);
        dual_exp.x() = q(5);
        dual_exp.y() = q(6);
        dual_exp.z() = q(7);

    }
  
    res_exp << prim_exp(0) ,prim_exp(1) ,prim_exp(2) ,prim_exp(3), 
               dual_exp.w(),dual_exp.x(),dual_exp.y(),dual_exp.z();

    return res_exp;

}

struct Point {
    double x, y, z;
};

struct TransformationCostFunction {
    TransformationCostFunction(const Eigen::Vector3d& source_, const Eigen::Vector3d& dest_)
        : source(source_), dest(dest_) {}

    template <typename T>
    bool operator()(const T* const parameters, T* residuals) const {

        Eigen::Matrix<T, 8, 1> Q0(parameters);

        Eigen::Matrix<T, 8, 1> Vi;

        Vi(0) = T(1.0);
        Vi(1) = T(0.0);
        Vi(2) = T(0.0);
        Vi(3) = T(0.0);
        Vi(4) = T(0.0);
        Vi(5) = T(source[0]);
        Vi(6) = T(source[1]);
        Vi(7) = T(source[2]);
        
        // Eigen::Matrix<T, 8, 1> difQ = dualquatMult(Q0,dualquatMult(Vi,dq_conjugate(Q0)));

        // residuals[0] = difQ[5] - dest[0];
        // residuals[1] = difQ[6] - dest[1];
        // residuals[2] = difQ[7] - dest[2];

        // std::cout<<"residuals: "<<residuals[0]<<std::endl;

        Eigen::Matrix<T, 8, 1> Q2 = log_dq(dualquatMult(dq_conjugate(Q0),Vi));

        residuals[0] = Q2[5] - dest[0];
        residuals[1] = Q2[6] - dest[1];
        residuals[2] = Q2[7] - dest[2];

        std::cout<<"residuals: "<<residuals[0]<<std::endl;

        return true;
    }

private:
    const Eigen::Vector3d source;
    const Eigen::Vector3d dest;
};


struct UDQManifold {

template<typename T>
    bool Plus(const T* parameters, const T* delta, T* x_plus_delta) const {

        Eigen::Matrix<T, 8, 1> udq(parameters);    


        T squared_norm_delta =  delta[0]*delta[0] + delta[1]*delta[1] + delta[2]*delta[2];
        
        std::cout<<"delta_data"<<delta[0]<<", "<<delta[1]<<", "<<delta[2]<<", "<<delta[3]<<", "<<delta[4]<<", "<<delta[5]<<std::endl;
        Eigen::Matrix<T, 4, 1> prim_exp;
        prim_exp << T(0.0) ,delta[0] ,delta[1] , delta[2];
 

        Eigen::Matrix<T, 4, 1> dual_exp;
         dual_exp << T(0.0) ,delta[3], delta[4], delta[5];

        if(squared_norm_delta != T(0.0)){

            T norm_delta = sqrt(squared_norm_delta);
            const T sin_delta_by_delta = sin(norm_delta) / norm_delta;
            prim_exp(0)  = cos(norm_delta) ;
            prim_exp(1) = sin_delta_by_delta*prim_exp(1); 
            prim_exp(2) = sin_delta_by_delta*prim_exp(2); 
            prim_exp(3) = sin_delta_by_delta*prim_exp(3); 
            dual_exp = quatMult_mT(dual_exp,prim_exp);
            //std::cout<<"diferente de 0"<<std::endl; 
            //std::cout<<"prim_exp: "<<prim_exp<<std::endl;
            //std::cout<<"dual_exp: "<<dual_exp<<std::endl;           
            
            
        }     
        else{
            prim_exp << T(1.0) ,delta[0] ,delta[1] , delta[2];
            dual_exp << T(0.0) ,delta[3] ,delta[4] , delta[5];
        }


        Eigen::Matrix<T,8,1> Q_delta;
        Q_delta << prim_exp(0) ,prim_exp(1) ,prim_exp(2) ,prim_exp(3), 
                   dual_exp(0) ,dual_exp(1),dual_exp(2),dual_exp(3);


        Eigen::Matrix<T,8,1> udq_plus_delta = dualquatMult(Q_delta,udq);

        x_plus_delta[0] = udq_plus_delta(0);
        x_plus_delta[1] = udq_plus_delta(1);
        x_plus_delta[2] = udq_plus_delta(2);
        x_plus_delta[3] = udq_plus_delta(3);
        x_plus_delta[4] = udq_plus_delta(4);
        x_plus_delta[5] = udq_plus_delta(5);
        x_plus_delta[6] = udq_plus_delta(6);
        x_plus_delta[7] = udq_plus_delta(7);

        return true;
    }


    template<typename T>
    bool Minus(const T* y, const T* parameters, T* y_minus_x) const {


        Eigen::Matrix<T, 8, 1> udq;
        udq << parameters[0],-parameters[1],-parameters[2],-parameters[3],parameters[4],-parameters[5],-parameters[6],-parameters[7];

        Eigen::Matrix<T, 8, 1> y_dq;
        y_dq << y[0],y[1],y[2],y[3],y[4],y[5],y[6],y[7];

        Eigen::Matrix<T,8,1> ambient_y_minus_x = dualquatMult(y_dq,udq);

        T u_norm = sqrt(ambient_y_minus_x[1] * ambient_y_minus_x[1] +
                        ambient_y_minus_x[2] * ambient_y_minus_x[2] +
                        ambient_y_minus_x[3] * ambient_y_minus_x[3]);


       if (u_norm > T(0.0)){
        
            T theta = atan2(u_norm,ambient_y_minus_x[0]);

            Eigen::Matrix<T,4,1> q;
            q << T(0.0), T(0.5)*theta*ambient_y_minus_x(1)/u_norm, 
                         T(0.5)*theta*ambient_y_minus_x(2)/u_norm,
                         T(0.5)*theta*ambient_y_minus_x(3)/u_norm;

            

        y_minus_x[0] = q[1];
        y_minus_x[1] = q[2];
        y_minus_x[2] = q[3];
            

         }
        else
        {
        y_minus_x[0] = ambient_y_minus_x[1];
        y_minus_x[1] = ambient_y_minus_x[2];
        y_minus_x[2] = ambient_y_minus_x[3];
            
        }
        
        Eigen::Matrix<T,4,1> d;
        d = T(0.5) * get_translation_dual(ambient_y_minus_x); //dual

            y_minus_x[3] = d[1];
            y_minus_x[4] = d[2];
            y_minus_x[5] = d[3];



         
        return true;
    }
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



int main() {
    
    ///////////////////////////////////////////// Cargando los datos /////////////////////////////////////
    // Especifica la ruta de tu archivo
    std::string sour_str = "/home/ceres/catkin_ws/src/ceres_example/points/source_bunny.txt";
    std::string dest_str = "/home/ceres/catkin_ws/src/ceres_example/points/dest_bunny.txt";

    Eigen::MatrixXf source_p = leerArchivoPuntos(sour_str);
    Eigen::MatrixXf dest_q = leerArchivoPuntos(dest_str);

  
    // // Inicializar la transformada homogenea
    // Eigen::Quaterniond quaternion = Eigen::Quaterniond::Identity();  // Inicialización del cuaternión
    // Eigen::Vector3d translation = Eigen::Vector3d::Zero();
    double parameters[8] = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // Configurar el problema de optimización de Ceres
    ceres::Problem problem;
    for (int k = 0; k < dest_q.rows(); ++k) {

        Eigen::Vector3d p = source_p.row(k).cast<double>(); 
        Eigen::Vector3d q = dest_q.row(k).cast<double>();
        //ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);

        ceres::Manifold* dq_manifold = new ceres::AutoDiffManifold<UDQManifold, 8, 6>;
            problem.AddParameterBlock(parameters, 8, dq_manifold);   

        ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<TransformationCostFunction, 3, 8>(
        new TransformationCostFunction(p, q));
        problem.AddResidualBlock(cost_function, nullptr, parameters); 


    }

    //Configurar opciones del optimizador
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 2000;
    options.gradient_check_relative_precision = 1e-6;
    options.function_tolerance = 1e-6;
    options.parameter_tolerance = 1e-6;

   

    // Resolver el problema de optimización
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // Mostrar resultados
    std::cout << summary.BriefReport() << std::endl;
    
    
    std::cout << "Quaternion Dual: " << parameters[0] << ", "<< parameters[1] << ", "<< parameters[2] << ", "<< parameters[3] << ", "
                                    << parameters[4] << ", "<< parameters[5] << ", "<< parameters[6] << ", "<< parameters[7] << std::endl;

    Eigen::Quaterniond q(1,0,0,0);
    Eigen::Matrix<double,3,1> t(1.0,2.0,3.0);
    Eigen::Matrix<double, 8, 1> datos = ToDQ_T(q,t);

                                                //Eigen::Matrix<double,8,1> ;
        Eigen::Matrix<double,4,1> res = get_translation_dual(datos);
        std::cout<<"resultado: "<<res.transpose();



    return 0;
}
