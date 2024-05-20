#include <iostream>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

// Estructura para representar los puntos en R3
struct Point {
    double x, y, z;
};

// Functor para el problema de optimización
struct PerpendicularDistanceCostFunction {
    PerpendicularDistanceCostFunction(const Point& P1, const Point& P2, const Point& P3)
        : P1(P1), P2(P2), P3(P3) {}

    template <typename T>
    bool operator()(const T* const quaternion, const T* const translation, T* residuals) const {
        // Transforma solo P3 usando la transformada homogénea
        Eigen::Quaternion<T> eigen_quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
        Eigen::Matrix<T, 3, 1> eigen_translation(translation[0], translation[1], translation[2]);

        Eigen::Matrix<T, 3, 1> transformed_P3;
        transformed_P3 << T(P3.x), T(P3.y), T(P3.z);
        transformed_P3 = eigen_quaternion * transformed_P3 + eigen_translation;
        
        Eigen::Matrix<T, 3, 1> P1_T;
        P1_T << T(P1.x),T(P1.y),T(P1.z);
        Eigen::Matrix<T, 3, 1> P2_T;
        P2_T << T(P2.x),T(P2.y),T(P2.z);
        Eigen::Matrix<T, 3, 1> P3_T;
        P3_T << T(P3.x),T(P3.y),T(P3.z);

        // Calcula el vector v1 y v2 
        Eigen::Matrix<T, 3, 1> v1;
        v1 << P2_T-P1_T;

        Eigen::Matrix<T, 3, 1> v2;
        v2 << P3_T-P1_T;

        // Calcula la proyección de P3 en la dirección de la recta P1-P2
        T projection = (v2).dot(v1) / pow(v1.norm(),2);

        // Calcula el punto de perpendicular de P3 a la recta que pasa por P1-P2


        Eigen::Matrix<T, 3, 1> pp = P1_T + projection * v1;

        // Calcula la distancia perpendicular entre P3 y la recta P1-P2
        residuals[0] = transformed_P3[0] - pp[0];
        residuals[1] = transformed_P3[1] - pp[1];
        residuals[2] = transformed_P3[2] - pp[2];


        return true;
    }

private:
    const Point P1, P2, P3;
};

int main() {
    // Definir puntos 
    Point P1 = {-0.20216, 0.249, 0.040749};
    Point P2 = {-0.97447, 0.11425, 0.041273};
    Point P3 = {0.0661, 0.2338, 0.40916};

    // Inicializar la transformada homogenea
    Eigen::Quaterniond quaternion(1.0, 0.0, 0.0, 0.0);  // w,x,y,z Identidad
    Eigen::Vector3d translation(0.0, 0.0, 0.0);         // Traslation 

////////////////////////////// Configuracion de Ceres 
    ceres::Problem problem;

    // Añadir la función de costo al problema
    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<PerpendicularDistanceCostFunction, 3, 4, 3>(
            new PerpendicularDistanceCostFunction(P1, P2, P3)),
        nullptr, quaternion.coeffs().data(), translation.data());

    // Configurar opciones del optimizador
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 6;
    options.gradient_check_relative_precision = 1e-4;

    ceres::Manifold* quaternion_parameterization = new ceres::EigenQuaternionManifold;
    problem.AddParameterBlock(quaternion.coeffs().data(), 4, quaternion_parameterization);

    // Resolver el problema de optimización
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // Mostrar resultados
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "Q = [ " << quaternion.coeffs().transpose() <<" ]"<< std::endl;
    std::cout << "T =  [" << translation.transpose() <<" ]"<< std::endl;

    return 0;
}
