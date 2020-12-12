#include <gtest/gtest.h>
#include <map_merge3d/EigenMatrix.h>
#include <Eigen/Eigen>

map_merge3d::EigenMatrix serializeMatrix(Eigen::MatrixXd& mat);

Eigen::MatrixXd deserializeMatrix(map_merge3d::EigenMatrix& mat);

TEST(TestSerialization, seriallizes_and_desrializes_correctly)
{
    Eigen::MatrixXd mat(10,20);
    double val =2;
    for(int i =0; i < mat.rows(); i++){
        for(int j =0; j<mat.cols();j++){
            mat(i,j) = val;
            val+=2;
        }
    }
    auto ser = serializeMatrix(mat);
    auto deser = deserializeMatrix(ser);
  
    ASSERT_EQ(mat, deser);
    
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
