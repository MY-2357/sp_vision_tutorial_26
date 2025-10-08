#include "io/my_camera.hpp"
#include "tasks/yolo.hpp"
#include "opencv2/opencv.hpp"
#include "tools/img_tools.hpp"

int main()
{
    // 初始化相机,yolo类
    myCamera camera;
    auto_aim::YOLO yolo("./configs/yolo.yaml");
    while (1)
    {
        // 调用相机读取图像
        cv::Mat img;
        if (!camera.read(img))
        {
            std::cout << "read image failed" << std::endl;
            return -1;
        }
        // 调用yolo识别装甲板
        auto draw_armor_name = [&img](auto_aim::Armor & armor)
        {
            std::string name;
            switch (armor.name)
            {
                case auto_aim::one: name="1"; break;
                case auto_aim::two: name="2"; break;
                case auto_aim::three: name="3"; break;
                case auto_aim::four: name="4"; break;
                case auto_aim::five: name="5"; break;
                case auto_aim::sentry: name="sentry"; break;
                case auto_aim::outpost: name="outpost"; break;
                case auto_aim::base: name="base"; break;
                case auto_aim::not_armor: name="not_armor"; break;
                default: name="unknown"; break;
            }
            tools::draw_text(img, name, armor.points[0], {0, 0, 255});
        };

        auto armors = yolo.detect(img);
        for(auto_aim::Armor armor : armors)
        {
            switch (armor.color)
            {
                case auto_aim::red:
                {
                    tools::draw_points(img, armor.points, {0, 0, 255});
                    tools::draw_text(img, "red", armor.center, {0, 0, 255});
                    draw_armor_name(armor);
                }
                break;
                case auto_aim::blue:
                {
                    tools::draw_points(img, armor.points, {0, 0, 255});
                    tools::draw_text(img, "blue", armor.center, {0, 0, 255});
                    draw_armor_name(armor);
                }
                break;
            } 
        }

        // 显示图像
        cv::resize(img, img, cv::Size(640, 480));
        cv::imshow("img", img);
        if (cv::waitKey(1) == 'q')      //备注：这个地方本来是0的，测试表明用0会卡在第一帧，不符合预期，故改用1
        {
            break;
        }
    }
    

    return 0;
}

// //测试用
// int main()
// {
//     // 初始化相机


//     // 初始化yolo类
//     auto_aim::YOLO yolo("./configs/yolo.yaml");
//     int n=0;
//     while (1)
//     {
//         cv::Mat img;
//         // 调用相机读取图像（模拟每次相机获取不同的图片的效果）
//         switch(n)
//         {
//             case 0:
//                 img=cv::imread("test_1.jpg");
//                 break;
//             case 1:
//                 img=cv::imread("test_2.jpg");
//                 break;
//             case 2:
//                 img=cv::imread("test_3.jpg");
//                 break;
//             default:
//                 img=cv::imread("test_1.jpg");
//                 n=0;
//         }
//         n++;

//         // 调用yolo识别装甲板
//         auto draw_armor_name = [&img](auto_aim::Armor & armor)
//         {
//             std::string name;
//             switch (armor.name)
//             {
//                 case auto_aim::one: name="1"; break;
//                 case auto_aim::two: name="2"; break;
//                 case auto_aim::three: name="3"; break;
//                 case auto_aim::four: name="4"; break;
//                 case auto_aim::five: name="5"; break;
//                 case auto_aim::sentry: name="sentry"; break;
//                 case auto_aim::outpost: name="outpost"; break;
//                 case auto_aim::base: name="base"; break;
//                 case auto_aim::not_armor: name="not_armor"; break;
//                 default: name="unknown"; break;
//             }
//             tools::draw_text(img, name, armor.points[0], {0, 0, 255});
//         };

//         auto armors = yolo.detect(img);
//         for(auto_aim::Armor armor : armors)
//         {
//             switch (armor.color)
//             {
//                 case auto_aim::red:
//                 {
//                     tools::draw_points(img, armor.points, {0, 0, 255});
//                     tools::draw_text(img, "red", armor.center, {0, 0, 255});
//                     draw_armor_name(armor);
//                 }
//                 break;
//                 case auto_aim::blue:
//                 {
//                     tools::draw_points(img, armor.points, {0, 0, 255});
//                     tools::draw_text(img, "blue", armor.center, {0, 0, 255});
//                     draw_armor_name(armor);
//                 }
//                 break;
//             } 
//         }

//         // 显示图像
//         cv::resize(img, img, cv::Size(640, 480));
//         cv::imshow("img", img);
//         if (cv::waitKey(1) == 'q')
//         {
//             break;
//         }
//     }

//     return 0;
// }
