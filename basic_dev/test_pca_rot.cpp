#include <Eigen/Dense>
#include <iostream>

int main() {
    Eigen::Matrix3f rot;
    rot << 1, 0, 0,
           0, 1, 0,
           0, 0, -1; // Det = -1
    std::cout << "Det: " << rot.determinant() << std::endl;
    Eigen::Quaternionf q(rot);
    std::cout << "Q: " << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << std::endl;
    return 0;
}
