#include <iostream>
#include <ctime>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;

int main(int argc, char** argv){
    //https://stackoverflow.com/questions/21292881/matrixxfrandom-always-returning-same-matrices
    srand((unsigned int) time(0));

    Eigen::Matrix<double, 100, 100> A = Eigen::Matrix<double, 100, 100>::Random(100, 100);
    Eigen::Matrix<double, 100, 1> b = Eigen::Matrix<double, 100, 1>::Random(100, 1);
    Eigen::Matrix<double, 100, 1> x;

    // make sure it's positive semidefinite so that we can use Cholesky decomposition
    A = A * A.transpose();

    cout << "A's determinant: " << A.determinant() << endl;
    assert(A.determinant() && "A is not invertable!");

    clock_t time_stt = clock();
    x = A.colPivHouseholderQr().solve(b);
    cout << "time of Qr decomposition is "
        << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << endl;
    cout << "x = " << x.transpose() << endl;

    time_stt = clock();
    // Eigen::LDLT: Robust Cholesky decomposition of a matrix with pivoting.
    x = A.ldlt().solve(b);
    cout << "time of ldlt decomposition is "
        << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << endl;
    cout << "x = " << x.transpose() << endl;
    return 0;
}