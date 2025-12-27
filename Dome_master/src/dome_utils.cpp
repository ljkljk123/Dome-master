#include "dome_utils.h"

// =======================
// 下面开始：
// 把你原文件中 main() 之前的
// 所有函数，原封不动粘到这里
// =======================

// loadAviaBin(...)
void loadAviaBin(const std::string &filename, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    std::ifstream input(filename, std::ios::binary);
    if (!input.good())
    {
        std::cerr << "Could not read file: " << filename << std::endl;
        return;
    }

    AviaPoint point;
    while (input.read(reinterpret_cast<char *>(&point), sizeof(AviaPoint)))
    {
        pcl::PointXYZI p;
        p.x = point.x;
        p.y = point.y;
        p.z = point.z;
        p.intensity = 1.0; // 使用反射强度
        cloud->push_back(p);
    }
    input.close();
}

// getGTFromPose(...)
std::vector<vector<int>> getGTFromPose(const string &pose_path)
{
    std::ifstream pose_ifs(pose_path);
    std::string line;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

    int index = 1;
    while (getline(pose_ifs, line))
    {
        if (line.empty())
            break;
        stringstream ss(line);
        float r1, r2, r3, t1, r4, r5, r6, t2, r7, r8, r9, t3;
        ss >> r1 >> r2 >> r3 >> t1 >> r4 >> r5 >> r6 >> t2 >> r7 >> r8 >> r9 >> t3;
        pcl::PointXYZI p;
        p.x = t1;
        p.y = t2;
        p.z = t3;
        p.intensity = index++;
        cloud->push_back(p);
    }

    pcl::io::savePCDFileASCII(seq + ".pcd", *cloud);
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(cloud);
    std::vector<vector<int>> res(30000);
    for (int i = 0; i < cloud->points.size(); i++)
    {
        float radius = 8;
        std::vector<int> ixi;
        std::vector<float> ixf;
        pcl::PointXYZI p = cloud->points[i];
        int cur = p.intensity;
        std::vector<int> nrs;
        kdtree.radiusSearch(p, radius, ixi, ixf);
        for (int j = 0; j < ixi.size(); j++)
        {
            if (cloud->points[ixi[j]].intensity == cur)
                continue;
            nrs.push_back(cloud->points[ixi[j]].intensity);
        }
        sort(nrs.begin(), nrs.end());
        res[cur] = nrs;
    }

    std::ofstream gt_ofs("/home/ros/TLG_LIO2/src/LinK3D-main/gth" + seq + ".txt");
    for (int i = 0; i < res.size(); i++)
    {
        gt_ofs << i << " ";
        for (int j = 0; j < res[i].size(); j++)
        {
            gt_ofs << res[i][j] << " ";
        }
        gt_ofs << endl;
    }
    return res;
}

// compute_global_lm_bin(...)
pair<int, int> compute_global_lm_bin(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud)
{
    const float radius_unit = 5.0f;
    const int num_radius_bins = int(MAX_RADIUS / radius_unit);
    const int num_yaw_bins = 360;

    vector<vector<int>> bin_count(num_yaw_bins, vector<int>(num_radius_bins, 0));

    for (const auto &pt : cloud->points)
    {
        float yaw = atan2(pt.y, pt.x) * 180.0f / M_PI;
        if (yaw < 0)
            yaw += 360;
        int yaw_idx = static_cast<int>(yaw);

        float radius = sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
        int r_bin = min(num_radius_bins - 1, int(radius / radius_unit));

        bin_count[yaw_idx][r_bin]++;
    }

    vector<pair<float, float>> lm_points;
    for (int yaw_idx = 0; yaw_idx < num_yaw_bins; yaw_idx++)
    {
        int max_r_idx = distance(bin_count[yaw_idx].begin(), max_element(bin_count[yaw_idx].begin(), bin_count[yaw_idx].end()));
        float yaw_rad = yaw_idx * M_PI / 180.0f;
        float radius = (max_r_idx + 0.5f) * radius_unit;
        float x = radius * cos(yaw_rad);
        float y = radius * sin(yaw_rad);
        lm_points.emplace_back(x, y);
    }

    float sum_x = 0.0f, sum_y = 0.0f;
    for (const auto &p : lm_points)
    {
        sum_x += p.first;
        sum_y += p.second;
    }

    float centroid_x = sum_x / lm_points.size();
    float centroid_y = sum_y / lm_points.size();

    float centroid_yaw = atan2(centroid_y, centroid_x) * 180.0f / M_PI;
    if (centroid_yaw < 0)
        centroid_yaw += 360;
    int centroid_yaw_bin = static_cast<int>(centroid_yaw);

    float centroid_radius = sqrt(centroid_x * centroid_x + centroid_y * centroid_y);
    int centroid_radius_bin = min(num_radius_bins - 1, int(centroid_radius / radius_unit));

    return {centroid_yaw_bin, centroid_radius_bin};
}

// generate_dome_image(...)
cv::Mat generate_dome_image(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, float phi_min, float phi_max)
{
    cv::Mat image = cv::Mat::zeros(NUM_PITCH_BINS, NUM_YAW_BINS, CV_32F); // 改为CV_32F
    map<pair<int, int>, vector<float>> bin_depths;

    for (const auto &pt : cloud->points)
    {
        float xy = sqrt(pt.x * pt.x + pt.y * pt.y);
        float pitch = atan2(pt.z, xy) * 180 / M_PI;
        int pitch_bin = min(NUM_PITCH_BINS - 1, max(0, int(((pitch - phi_min) / (phi_max - phi_min)) * NUM_PITCH_BINS)));
        // float phi = atan2(pt.z, xy); // rad
        // float y_merc = log(tan(M_PI / 4.0f + phi / 2.0f));
        // float y_min = log(tan(M_PI / 4.0f + phi_min / 2.0f));
        // float y_max = log(tan(M_PI / 4.0f + phi_max / 2.0f));
        // float y_norm = (y_merc - y_min) / (y_max - y_min);
        // int pitch_bin = std::clamp(int(y_norm * NUM_PITCH_BINS), 0, NUM_PITCH_BINS - 1);

        float yaw = atan2(pt.y, pt.x) * 180 / M_PI;
        if (yaw < 0)
            yaw += 360;
        int yaw_bin = min(NUM_YAW_BINS - 1, int(yaw / 360.0 * NUM_YAW_BINS));

        float abs_depth = sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
        bin_depths[{pitch_bin, yaw_bin}].push_back(abs_depth);
    }

    for (const auto &[bin, depths] : bin_depths)
    {
        vector<float> sorted_depths = depths;
        sort(sorted_depths.begin(), sorted_depths.end());
        float avg_depth = (depths.size() >= 3)
                              ? accumulate(sorted_depths.end() - 3, sorted_depths.end(), 0.0f) / 3.0f
                              : accumulate(sorted_depths.begin(), sorted_depths.end(), 0.0f) / depths.size();

        float weight = log(depths.size() + 1);
        float intensity = avg_depth * weight;
        image.at<float>(bin.first, bin.second) = intensity / MAX_RADIUS * 255.0f; // 归一到255
    }

    return image;
}

// convolve_with_pitch_weight(...)
cv::Mat convolve_with_pitch_weight(const cv::Mat &py_image, const cv::Mat &pitch_weight)
{
    // 将 pitch_weight 转为卷积核（列向量）
    cv::Mat kernel = pitch_weight.clone(); // pitch_weight 是 (pitch_bins, 1)

    // 执行2D卷积，边界填0，效果等同于 convolve2d(..., mode='same', boundary='fill', fillvalue=0)
    cv::Mat convolved;
    cv::filter2D(py_image, convolved, -1, kernel, cv::Point(-1, -1), 0, cv::BORDER_CONSTANT);

    return convolved;
}

// compute_yaw_weight(...)
cv::Mat compute_yaw_weight(const cv::Mat &expanded_py, float epsilon)
{
    cv::Mat expanded_py_float;
    expanded_py.convertTo(expanded_py_float, CV_32F);

    cv::Mat yaw_sum;
    reduce(expanded_py_float, yaw_sum, 0, CV_REDUCE_SUM, CV_32F);

    float total = sum(yaw_sum)[0] + epsilon;
    yaw_sum /= total;

    return yaw_sum;
}

// generate_ry_max_height(...)
cv::Mat generate_ry_max_height(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud)
{
    cv::Mat ry_height = cv::Mat::ones(NUM_RADIUS_BINS, NUM_YAW_BINS, CV_32F) * -INFINITY;

    for (const auto &pt : cloud->points)
    {
        float radius = sqrt(pt.x * pt.x + pt.y * pt.y); // 使用 xy 平面距离
        if (radius >= MAX_RADIUS)
            continue;
        int r_bin = min(NUM_RADIUS_BINS - 1, int(radius / (MAX_RADIUS / NUM_RADIUS_BINS)));

        float yaw = atan2(pt.y, pt.x) * 180 / M_PI;
        if (yaw < 0)
            yaw += 360;
        int y_bin = min(NUM_YAW_BINS - 1, int(yaw / 360.0 * NUM_YAW_BINS));

        float z_height = pt.z;
        ry_height.at<float>(r_bin, y_bin) = max(ry_height.at<float>(r_bin, y_bin), z_height);
    }

    // 把 -inf 置0
    for (int r = 0; r < ry_height.rows; ++r)
    {
        for (int y = 0; y < ry_height.cols; ++y)
        {
            if (ry_height.at<float>(r, y) == -INFINITY)
                ry_height.at<float>(r, y) = 0.0f;
        }
    }
    return ry_height;
}

// compute_1d_pitch_weight(...)
cv::Mat compute_1d_pitch_weight(const cv::Mat &rp_density, float epsilon)
{
    cv::Mat rp_density_float;
    rp_density.convertTo(rp_density_float, CV_32F);
    cv::Mat pitch_weight;
    reduce(rp_density_float, pitch_weight, 0, CV_REDUCE_SUM, CV_32F);

    float total = sum(pitch_weight)[0] + epsilon;
    pitch_weight /= total;
    return pitch_weight;
}

// preprocess_pointcloud(...)
tuple<pcl::PointCloud<pcl::PointXYZI>::Ptr, float, float> preprocess_pointcloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, float ground_th)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());
    vector<float> pitches;
    for (const auto &pt : cloud->points)
    {
        if (pt.z > ground_th)
        {
            filtered->push_back(pt);
            float xy_dist = sqrt(pt.x * pt.x + pt.y * pt.y);
            float pitch = atan2(pt.z, xy_dist) * 180 / M_PI;
            pitches.push_back(pitch);
        }
    }
    float phi_min = -30.0f;
    float phi_max = 10.0f;
    if (!pitches.empty())
    {
        auto [min_it, max_it] = minmax_element(pitches.begin(), pitches.end());
        phi_min = *min_it;
        phi_max = *max_it;
    }
    return make_tuple(filtered, phi_min, phi_max);
}

// extract_dcl_features(...)
vector<float> extract_dcl_features(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, float phi_min, float phi_max)
{
    // auto global_bin = compute_global_lm_bin(cloud);
    auto dome_image = generate_dome_image(cloud, phi_min, phi_max);

    cv::Mat rp_density = cv::Mat::zeros(NUM_RADIUS_BINS, NUM_PITCH_BINS, CV_32F);
    for (const auto &pt : cloud->points)
    {
        float radius = sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
        int r_bin = min(NUM_RADIUS_BINS - 1, int(radius / (MAX_RADIUS / NUM_RADIUS_BINS)));

        float xy = sqrt(pt.x * pt.x + pt.y * pt.y);
        // float phi = atan2(pt.z, xy); // rad
        // float y_merc = log(tan(M_PI / 4.0f + phi / 2.0f));
        // float y_min = log(tan(M_PI / 4.0f + phi_min / 2.0f));
        // float y_max = log(tan(M_PI / 4.0f + phi_max / 2.0f));
        // float y_norm = (y_merc - y_min) / (y_max - y_min);
        // int pitch_bin = std::clamp(int(y_norm * NUM_PITCH_BINS), 0, NUM_PITCH_BINS - 1);
        float pitch = atan2(pt.z, xy) * 180 / M_PI;
        int pitch_bin = clamp(int((pitch - phi_min) / (phi_max - phi_min) * NUM_PITCH_BINS), 0, NUM_PITCH_BINS - 1);

        rp_density.at<float>(r_bin, pitch_bin) += 1.0f;
    }

    auto pitch_weight = compute_1d_pitch_weight(rp_density);
    auto expanded_py = convolve_with_pitch_weight(dome_image, pitch_weight);
    auto yaw_weight = compute_yaw_weight(expanded_py);
    auto ry_height = generate_ry_max_height(cloud);

    cv::Mat temp_weight;
    cv::repeat(yaw_weight, ry_height.rows, 1, temp_weight); // 重复 yaw_weight 行数倍，匹配 ry_height 尺寸
    cv::Mat weighted_ry = ry_height.mul(temp_weight);       // 按元素相乘

    // double max_t, min_t;
    // cv::minMaxLoc(weighted_ry, &min_t, &max_t);
    // std::cout << "weighted_ry" << " max: " << max_t << " min:" << min_t << endl;

    vector<float> flat;
    flat.assign((float *)weighted_ry.datastart, (float *)weighted_ry.dataend);
    return flat;
}

// normalize_vector(...)
void normalize_vector(vector<float> &v)
{
    double mean = accumulate(v.begin(), v.end(), 0.0) / v.size();
    double variance = 0.0;
    for (const auto &val : v)
    {
        variance += (val - mean) * (val - mean);
    }
    double stddev = sqrt(variance / v.size()) + 1e-8;

    for (auto &val : v)
    {
        val = (val - mean) / stddev;
    }
}

// compute_cosine_similarity_fft(...)
double compute_cosine_similarity_fft(const vector<float> &V1, const vector<float> &V2)
{
    vector<float> v1 = V1;
    vector<float> v2 = V2;
    normalize_vector(v1);
    normalize_vector(v2);

    int N = v1.size();
    fftw_complex *in1 = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * N);
    fftw_complex *in2 = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * N);
    fftw_complex *fft1 = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * N);
    fftw_complex *fft2 = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * N);
    fftw_complex *ifft = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * N);

    for (int i = 0; i < N; ++i)
    {
        in1[i][0] = v1[i];
        in1[i][1] = 0.0;
        in2[i][0] = v2[i];
        in2[i][1] = 0.0;
    }

    auto p1 = fftw_plan_dft_1d(N, in1, fft1, FFTW_FORWARD, FFTW_ESTIMATE);
    auto p2 = fftw_plan_dft_1d(N, in2, fft2, FFTW_FORWARD, FFTW_ESTIMATE);
    fftw_execute(p1);
    fftw_execute(p2);

    // 频域相乘，fft1 * conj(fft2)
    for (int i = 0; i < N; ++i)
    {
        double real1 = fft1[i][0], imag1 = fft1[i][1];
        double real2 = fft2[i][0], imag2 = fft2[i][1];

        ifft[i][0] = real1 * real2 + imag1 * imag2; // 实部
        ifft[i][1] = imag1 * real2 - real1 * imag2; // 虚部
    }

    auto p_inv = fftw_plan_dft_1d(N, ifft, ifft, FFTW_BACKWARD, FFTW_ESTIMATE);
    fftw_execute(p_inv);

    double max_corr = 0.0;
    for (int i = 0; i < N; ++i)
    {
        double val = ifft[i][0] / N; // 取实部并归一化
        if (val > max_corr)
            max_corr = val;
    }

    fftw_destroy_plan(p1);
    fftw_destroy_plan(p2);
    fftw_destroy_plan(p_inv);
    fftw_free(in1);
    fftw_free(in2);
    fftw_free(fft1);
    fftw_free(fft2);
    fftw_free(ifft);

    return max_corr;
}

// compute_yaw_fft(...)
void compute_yaw_fft(
    const vector<float> &featA,
    const vector<float> &featB,
    double &out_max_corr,
    double &out_yaw_deg)
{
    // 1. 先从 flat feature 里聚合出 yaw profile
    vector<double> profA(NUM_YAW_BINS, 0.0), profB(NUM_YAW_BINS, 0.0);
    for (int r = 0; r < NUM_RADIUS_BINS; ++r)
    {
        for (int y = 0; y < NUM_YAW_BINS; ++y)
        {
            int idx = r * NUM_YAW_BINS + y;
            profA[y] += featA[idx];
            profB[y] += featB[idx];
        }
    }

    // 2. 可选：归一化（零均值+单位方差）
    auto normalize = [](vector<double> &v)
    {
        double mean = accumulate(v.begin(), v.end(), 0.0) / v.size();
        double var = 0.0;
        for (auto &x : v)
            var += (x - mean) * (x - mean);
        double stddev = sqrt(var / v.size()) + 1e-8;
        for (auto &x : v)
            x = (x - mean) / stddev;
    };
    normalize(profA);
    normalize(profB);

    int N = NUM_YAW_BINS;
    fftw_complex *in1 = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * N);
    fftw_complex *in2 = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * N);
    fftw_complex *fft1 = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * N);
    fftw_complex *fft2 = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * N);
    fftw_complex *ifft = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * N);

    for (int i = 0; i < N; ++i)
    {
        in1[i][0] = profA[i];
        in1[i][1] = 0.0;
        in2[i][0] = profB[i];
        in2[i][1] = 0.0;
    }

    auto p1 = fftw_plan_dft_1d(N, in1, fft1, FFTW_FORWARD, FFTW_ESTIMATE);
    auto p2 = fftw_plan_dft_1d(N, in2, fft2, FFTW_FORWARD, FFTW_ESTIMATE);
    fftw_execute(p1);
    fftw_execute(p2);

    // 3. 频域相乘：fft1 * conj(fft2)
    for (int i = 0; i < N; ++i)
    {
        double a = fft1[i][0], b = fft1[i][1];
        double c = fft2[i][0], d = fft2[i][1];
        // (a+jb) * (c-jd) = (ac+bd) + j(bc-ad)
        ifft[i][0] = a * c + b * d;
        ifft[i][1] = b * c - a * d;
    }

    auto p_inv = fftw_plan_dft_1d(N, ifft, ifft, FFTW_BACKWARD, FFTW_ESTIMATE);
    fftw_execute(p_inv);

    // 4. 在相关序列里找峰值及其位置
    int peak_idx = 0;
    double peak_val = -1e9;
    for (int k = 0; k < N; ++k)
    {
        double val = ifft[k][0] / N; // 实部归一化
        if (val > peak_val)
        {
            peak_val = val;
            peak_idx = k;
        }
    }

    // 5. 把 index 变成 shift（[-N/2, N/2]）
    int shift = peak_idx;
    if (shift > N / 2)
        shift -= N; // wrap 到负方向

    // yaw: 列方向一共 N 个 bin -> 360°
    double yaw_per_bin = 360.0 / double(N);

    // 注意符号：这里假设 "把B向右循环平移 shift 列可对齐A"
    out_yaw_deg = -shift * yaw_per_bin;
    out_max_corr = peak_val;

    fftw_destroy_plan(p1);
    fftw_destroy_plan(p2);
    fftw_destroy_plan(p_inv);
    fftw_free(in1);
    fftw_free(in2);
    fftw_free(fft1);
    fftw_free(fft2);
    fftw_free(ifft);
}

// plane_clip(...)
pcl::PointCloud<pcl::PointXYZI>::Ptr plane_clip(const pcl::PointCloud<pcl::PointXYZI>::Ptr &src_cloud, const Eigen::Vector4f &plane, bool negative)
{
    pcl::PlaneClipper3D<pcl::PointXYZI> clipper(plane);
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    clipper.clipPointCloud3D(*src_cloud, indices->indices);
    pcl::PointCloud<pcl::PointXYZI>::Ptr dst_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(src_cloud);
    extract.setIndices(indices);
    extract.setNegative(negative);
    extract.filter(*dst_cloud);
    return dst_cloud;
}

// normal_filtering(...)
pcl::PointCloud<pcl::PointXYZI>::Ptr normal_filtering(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, double sensor_height, double normal_filter_thresh)
{
    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    ne.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ne.setKSearch(10);
    ne.setViewPoint(0.0f, 0.0f, sensor_height);
    ne.compute(*normals);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>);
    filtered->reserve(cloud->size());
    for (int i = 0; i < cloud->size(); i++)
    {
        float dot = normals->at(i).getNormalVector3fMap().normalized().dot(Eigen::Vector3f::UnitZ());
        if (std::abs(dot) > std::cos(normal_filter_thresh * M_PI / 180.0))
        {
            filtered->push_back(cloud->at(i)); // filtered即为筛选过的地面点云
        }
    }
    filtered->width = filtered->size();
    filtered->height = 1;
    filtered->is_dense = false;
    return filtered;
}

// detect(...)
pcl::PointCloud<pcl::PointXYZI>::Ptr detect(pcl::PointCloud<pcl::PointXYZI> cloud)
{
    double sensor_height = 0.0;         // 近似传感器高度 [m]
    double normal_filter_thresh = 20.0; // “非”垂直度检查阈值 [deg]
    double height_clip_range = 0.2;     // 64线:0.2     高度在[sensor_height_clip_range，sensor_height+height_clip_range]内的点将用于地板检测
    Eigen::Matrix4f tilt_matrix = Eigen::Matrix4f::Identity();
    double tilt_deg = 0.0;
    tilt_matrix.topLeftCorner(3, 3) = Eigen::AngleAxisf(tilt_deg * M_PI / 180.0f, Eigen::Vector3f::UnitY()).toRotationMatrix();
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>);  // 存储滤波后的点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered2(new pcl::PointCloud<pcl::PointXYZI>); // 存储滤波后的点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_point(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(cloud, *filtered, tilt_matrix);                         // 出于对传感器安装角度的考虑，将点云位姿进行调整
    filtered2 = plane_clip(filtered, Eigen::Vector4f(0.0f, 0.0f, 1.0f, 0.3), false); // 保留裁剪平面上方的点，而移除平面以下的点
    pcl::transformPointCloud(*filtered2, *filtered2, static_cast<Eigen::Matrix4f>(tilt_matrix.inverse()));
    return filtered2;
}

// computeElevationRange(...)
std::pair<float, float> computeElevationRange(const pcl::PointCloud<pcl::PointXYZI> &cloud)
{
    float phi_min = 999.0f, phi_max = -999.0f;
    for (const auto &pt : cloud.points)
    {
        float xy = std::sqrt(pt.x * pt.x + pt.y * pt.y);
        float phi = std::atan2(pt.z, xy) * 180.0f / CV_PI;
        phi_min = std::min(phi_min, phi);
        phi_max = std::max(phi_max, phi);
    }
    return {phi_min, phi_max};
}