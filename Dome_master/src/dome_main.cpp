#include "dome_utils.h"

#include <ros/ros.h>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <iomanip>



int main(int argc, char *argv[])
{
    ros::init(argc, argv, "LinK3D_loop_test_dome");
    ros::NodeHandle nh;

    auto gt = getGTFromPose("/home/ros/TLG_LIO2/src/LinK3D-main/picture_k05/05.txt");
    std::ofstream ofs_solid("/home/ros/TLG_LIO2/src/LinK3D-main/picture_k05/guthub/test_res_dome_" + seq + ".txt");

    std::unordered_map<int, std::vector<float>> database;

    for (int i = 0; i < N; ++i)
    {
        std::stringstream ss;
        ss << setw(10) << setfill('0') << i;
        cout << ss.str() << endl;
        std::string filename = "/media/ros/ljk/KITTI/kitti_05/data/" + ss.str() + ".bin";
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud0(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZI>);

        // velodyne
        /////////////////////////////////////////////////////////////////////////////////////
        std::fstream input(filename, std::ios::in | std::ios::binary);
        input.seekg(0, std::ios::beg);
        for (int ii = 0; input.good() && !input.eof(); ii++)
        {
            pcl::PointXYZ point;
            pcl::PointXYZI point1;
            input.read((char *)&point.x, 3 * sizeof(float));
            float intensity;
            input.read((char *)&intensity, sizeof(float));
            point1.x = point.x;
            point1.y = point.y;
            point1.z = point.z;
            point1.intensity = 1.0;
            cloud0->push_back(point);
            cloud1->push_back(point1);
        }

        // Avia
        // loadAviaBin(filename, cloud1);

        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered = detect(*cloud1);
        auto [phi_min1, phi_max1] = computeElevationRange(*filtered);
        auto feature = extract_dcl_features(filtered, phi_min1, phi_max1);
        database[i] = feature;

        int matched_id = -1;
        double max_similarity = 0.0;
        double max_yaw = 0.0;
        for (int j = 0; j < i - 300; ++j)
        {
            double similarity = compute_cosine_similarity_fft(feature, database[j]);

            // double similarity = 0.0;
            // double yaw = 0.0;
            // compute_yaw_fft(feature, database[j], similarity, yaw);

            if (similarity > max_similarity)
            {
                max_similarity = similarity;
                // max_yaw = yaw;
                matched_id = j;
            }
        }

        // 请注意此处输出为带有yaw角的版本
        if (matched_id >= 0 && find(gt[i + 1].begin(), gt[i + 1].end(), matched_id + 1) != gt[i + 1].end())
        {
            ofs_solid << i + 1 << " " << matched_id + 1 << " " << max_similarity << " " << 1 << std::endl;
            cout << i + 1 << " " << matched_id + 1 << " " << max_similarity << " " << 1 << std::endl;
            // ofs_solid << i + 1 << " " << matched_id + 1 << " " << max_similarity << " " << max_yaw << " " << 1 << std::endl;
            // cout << i + 1 << " " << matched_id + 1 << " " << max_similarity << " " << max_yaw << " " << 1 << std::endl;
        }
        else
        {
            ofs_solid << i + 1 << " " << matched_id + 1 << " " << max_similarity << " " << 0 << std::endl;
            cout << i + 1 << " " << matched_id + 1 << " " << max_similarity << " " << 0 << std::endl;
            // ofs_solid << i + 1 << " " << matched_id + 1 << " " << max_similarity << " " << max_yaw << " " << 0 << std::endl;
            // cout << i + 1 << " " << matched_id + 1 << " " << max_similarity << " " << max_yaw << " " << 0 << std::endl;
        }
    }
    ros::spin();
    ros::shutdown();
    return 0;
}
