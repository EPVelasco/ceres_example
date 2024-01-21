#include <iostream>
#include <fstream>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

///////////////////////////////////////////////// FUNCIONES QUE LUEGO VAN EN LA LIBRERIA/////////////////////////////////////////////////////////
// Funcion para leer los puntos del bunny ( el formato del archivo es .txt y deben tener los puntos en cada linea un punto xyz)

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


struct Point {
    double x, y, z;
};

struct TransformationCostFunction {
    TransformationCostFunction(const Eigen::Vector3d& source, const Eigen::Vector3d& dest)
        : source(source), dest(dest) {}

    template <typename T>
    bool operator()(const T* const quaternion, const T* const translation, T* residuals) const {
        Eigen::Quaternion<T> eigen_quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
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


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



int main() {
    
    ///////////////////////////////////////////// Cargando los datos /////////////////////////////////////
    // Especifica la ruta de tu archivo
    std::string sour_str = "/home/ceres/catkin_ws/src/ceres_example/points/source_bunny.txt";
    std::string dest_str = "/home/ceres/catkin_ws/src/ceres_example/points/dest_bunny.txt";

    Eigen::MatrixXf source_p = leerArchivoPuntos(sour_str);
    Eigen::MatrixXf dest_q = leerArchivoPuntos(dest_str);

    // Imprime la matriz de coordenadas
    // std::cout << "Matriz de Origen :\n" << sour_bunny << std::endl;
    // std::cout << "Matriz de Destino:\n" << dest_bunny << std::endl;
    //////////////////////////////////////////////////////////////////////////////////////////////////////

    // ///////////////////////////////////////////// Prerpocesado ///////////////////////////////////////////

     //int npts = source_p.rows(); // cantidad de puntos 

    // // Calcular las medias de las nubes de puntos
    // Eigen::VectorXd p_m = p.colwise().mean().cast<double>();
    // Eigen::VectorXd q_m = q.colwise().mean().cast<double>();

    // std::cout<<"media pm:"<<p_m<<std::endl;
    // std::cout<<"media qm:"<<q_m<<std::endl;

    // // Nuevos datos
    // //Eigen::MatrixXd pi = p - p_m.transpose().replicate(npts, 1);
    // //Eigen::MatrixXd qi = q - q_m.transpose().replicate(npts, 1);

    // // Valor inicial de M
    // Eigen::Matrix4d M = Eigen::Matrix4d::Zero();

    // //////////////////////////////////////////////////////////////////////////////////////////////////////

    // /////////////////////////////////////// Calculo de suma //////////////////////////////////////////////
    // for (int k = 0; k < npts; ++k) {
    //     Eigen::Vector3d pi = p.row(k).cast<double>() - p_m.transpose();
    //     Eigen::Vector3d qi = q.row(k).cast<double>() - q_m.transpose();

    //     Eigen::Matrix4d Pi;
    //     Pi << 0,     -pi(0), -pi(1), -pi(2),
    //           pi(0),      0,  pi(2), -pi(1),
    //           pi(1), -pi(2),      0,  pi(0),
    //           pi(2),  pi(1), -pi(0),     0;

    //     Eigen::Matrix4d Qi;
    //     Qi << 0,     -qi(0), -qi(1), -qi(2),
    //           qi(0),      0, -qi(2),  qi(1),
    //           qi(1),  qi(2),      0, -qi(0),
    //           qi(2), -qi(1),  qi(1),      0;

    //     M += Pi.transpose() * Qi;
    // }

    // std::cout<<"M: "<<M<<std::endl;

    // ////////////////////////////////////////////////////////////////////////////////////////////////////


    // Inicializar la transformada homogenea
    Eigen::Quaterniond quaternion(1.0, 0.0, 0.0, 0.0);  // w,x,y,z Identidad
    Eigen::Vector3d translation(0.0, 0.0, 0.0);         // Traslation 

    // Configurar el problema de optimización de Ceres
    ceres::Problem problem;
    for (int k = 0; k < dest_q.rows(); ++k) {

        Eigen::Vector3d p = source_p.row(k).cast<double>(); 
        Eigen::Vector3d q = dest_q.row(k).cast<double>();

        ceres::CostFunction* cost_function =
            new ceres::AutoDiffCostFunction<TransformationCostFunction, 3, 4, 3>(
                new TransformationCostFunction(p, q));
        problem.AddResidualBlock(cost_function, nullptr, quaternion.coeffs().data(), translation.data());
    }

    //Configurar opciones del optimizador
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 2000;
    options.gradient_check_relative_precision = 1e-6;
    options.function_tolerance = 1e-6;
    options.parameter_tolerance = 1e-6;

    ceres::Manifold* quaternion_parameterization = new ceres::EigenQuaternionManifold;
    problem.AddParameterBlock(quaternion.coeffs().data(), 4, quaternion_parameterization);

    // Resolver el problema de optimización
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // Mostrar resultados
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "Optimized Quaternion: " << quaternion.coeffs().transpose() << std::endl;
    std::cout << "Optimized Translation: " << translation.transpose() << std::endl;

    //Calcular los puntos transformados con la solución óptima
    //Eigen::MatrixXd transformed_points = quaternion_left(quaternion) * quaternion_right(conjugate_quaternion(quaternion)) * source_p;
    //transformed_points.colwise() += translation;


    return 0;
}
