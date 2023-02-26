#include <iostream>
#include <ceres/ceres.h>
#include "glog/logging.h"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

using namespace std;

//1. 定义CostFuntion
struct CostFunctor{
    template<typename T>
    bool operator()(const T* const x, T*residual)const{
        residual[0] = T(10.0) - x[0];
        return true;
    }
};

void problem1()
{
    //初始化
    double init_x = 5.0;
    double x = init_x;
    //2. 构建问题
    Problem problem;
	
	//设置目标函数，AutoDiffCostFunction将刚刚建立的CostFunctor 结构的一个实例作为输入，自动生成其微分并且赋予其一个CostFunction 类型的接口
    CostFunction* cost_function =
            new AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);
   //添加误差项，调用AddResidualBlock将误差添加到目标函数中
    problem.AddResidualBlock(cost_function, nullptr, &x);

	//3. 调用solve函数求解，在options里可以配置各种优化的选项，可以选择Line Search或者Trust Region、迭代次数、步长等等
    Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    
	//4. 定义Summary
    Solver::Summary summary;
    
    //5. 开始优化Solve
    Solve(options, &problem, &summary);
    
	//6.输出结果SUmmary.BriefReport
    cout<<summary.BriefReport()<<"\n";
    cout<<"x: "<<init_x<<" -> "<<x<<"\n";
}

int main(int argc, char** argv) 
{
    problem1();
    return 0;
}

