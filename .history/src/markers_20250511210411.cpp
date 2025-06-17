#include "markers.hpp"
#include "camera.hpp" 
MarkersManager::MarkersManager(Utils::Options::MarkersSetup setup, Camera *cam) {
    config = setup;
    cam_obj = cam;
}

std::vector<std::shared_ptr<Marker>> MarkersManager::matchMarkers(std::vector<double> frequencies,
                                                 std::vector<std::vector<cv::Point>> filtered_contours,
                                                 long long detection_time)
{
    std::vector<std::pair<std::vector<int>, int>> output;
    std::vector<int> matches(frequencies.size(), 0);
//    std::vector<Marker> detected_markers;
    std::vector<std::shared_ptr<Marker>> detected_markers;

    for (unsigned int i = 0; i < config.ids.size(); ++i){
        std::vector<int> assigned(config.frequencies[i].size(), -1);
        std::vector<std::vector<cv::Point>> assigned_contours(config.frequencies[i].size());
        std::vector<int> assigned_frequencies(config.frequencies[i].size(), -1);
        bool not_found = false;

        // for (auto freq : frequencies){
        //     std::cout << freq << " ";
        // }
        // std::cout << std::endl;    

        for (unsigned int j = 0; j < config.frequencies[i].size(); ++j){
            // find frequency in the list of frequencies if the distance is less than 20 Hz
            // write functions which will return the index of the frequency in the list if the distance between query and object is below threshold

            //auto it = std::find(frequencies.begin(), frequencies.end(), config.frequencies[i][j]);

            auto it = std::find_if(frequencies.begin(), frequencies.end(), [&](int freq){
                return abs(freq - config.frequencies[i][j]) < 25;
            }); 
            if (it != frequencies.end())
            {
                int index = it - frequencies.begin();
                assigned[j] = index;
            }
            else{
                // std::cout << "not found" << config.frequencies[i][j] << std::endl;
                not_found = true;
                break;
            }
        }

        if (!not_found){
            output.push_back(std::make_pair(assigned, i));
            for (unsigned int j = 0; j<assigned.size(); ++j){
                assigned_contours[j] = filtered_contours[assigned[j]];
                assigned_frequencies[j] = frequencies[assigned[j]];
            }

        

            detected_markers.push_back(std::make_shared<Marker>(config.ids[i],
                                                                config.coordinates[i],
                                                                assigned_contours,
                                                                assigned_frequencies,
                                                                detection_time,
                                                                this->cam_obj,
                                                                logger_ptr,
                                                                this));
        }
    }
    return detected_markers;
}

void MarkersManager::registerDetections(std::vector<double> frequencies,
                                        std::vector<std::vector<cv::Point>> filtered_contours,
                                        long long int detection_timestamp) {
    auto detected_markers = matchMarkers(frequencies, filtered_contours, detection_timestamp);

    tracked_markers_ids_mutex.lock();
    for (auto &marker : detected_markers){
//        std::cout << tracked_markers_ids.size() << std::endl;
        auto it = std::find(tracked_markers_ids.begin(), tracked_markers_ids.end(), marker->id);
        if (it != tracked_markers_ids.end()) {

        }
        else{
            tracked_markers_ids.push_back(marker->id);
            markers_to_spawn.push_back(marker);
        }
    }
    tracked_markers_ids_mutex.unlock();
};

void MarkersManager::deregisterMarker(Marker *marker_to_remove) {
    tracked_markers_ids_mutex.lock();
    auto it = std::find(tracked_markers_ids.begin(), tracked_markers_ids.end(), marker_to_remove->id);
    if (it != tracked_markers_ids.end()) {
        tracked_markers_ids.erase(it);
    }
    tracked_markers_ids_mutex.unlock();
    tracked_markers_mutex.lock();
    for(size_t i = 0; i < tracked_markers.size(); i++){
        if (tracked_markers[i].get() == marker_to_remove){
            markers_to_destroy.push_back(tracked_markers[i]);
            tracked_markers_mutex.unlock();
            break;
        }
    }
}

void MarkersManager::spawnMarkers(std::shared_ptr<Deque> deque, Buffers *input_buffers){
    tracked_markers_mutex.lock();

    for (auto &marker : markers_to_destroy){
        auto it = std::find(tracked_markers.begin(), tracked_markers.end(), marker);
        if (it != tracked_markers.end()) {
            tracked_markers.erase(it);
        }
    }
    markers_to_destroy.clear();
    for (auto &marker : markers_to_spawn){

        tracked_markers.push_back(marker);
        tracked_markers.back()->startTracking(deque, input_buffers);
    }
    tracked_markers_mutex.unlock();
    markers_to_spawn.clear();

}

std::shared_ptr<OutputEntry> Marker::getOutput(bool detection = false) {
    current_output = std::make_shared<OutputEntry>();
    current_output->id = id;
    current_output->camera_timestamp = current_camera_timestamp;
    current_output->pc_timestamp = current_pc_timestamp;
    current_output->detection = detection;
    current_output->pose_trans.x = t_vec.at<double>(0);
    current_output->pose_trans.y = t_vec.at<double>(1);
    current_output->pose_trans.z = t_vec.at<double>(2);
    current_output->pose_rot.r_0 = r_vec.at<double>(0);
    current_output->pose_rot.r_1 = r_vec.at<double>(1);
    current_output->pose_rot.r_2 = r_vec.at<double>(2);

    return current_output;
}


void Marker::startTracking(std::shared_ptr<Deque> deque, Buffers *input_buffers) {
    initial_buffer = deque;
    input_buffers_ptr = input_buffers;
    buffers.setInputBuffer(input_buffers_ptr->getOutputBuffer());
    tracking_thread = std::make_shared<std::jthread>(&Marker::track, this);
}

void Marker::stopTracking(){
    input_buffers_ptr->deregisterBuffer(buffers.getInputBuffer());
}

void Marker::track() {
    std::cout << "[Marker ID: " << id << " TRACK] Detected - starting tracking" << std::endl << std::flush;

    // --- 调试和检查初始PnP的点 ---
    std::cout << "[Marker ID: " << id << " TRACK_INIT_PNP] Attempting initial PnP. "
              << "Object points count: " << objectpoints_3d.size()
              << ", Detected initial 2D points count: " << detected_initial_points.size()
              << std::endl << std::flush;

    bool initial_pnp_success = false;
    // 确保3D点和2D点的数量匹配，并且至少有3个点 (SQPNP的要求，通常4个或更多更好)
    if (objectpoints_3d.size() == detected_initial_points.size() && objectpoints_3d.size() >= 3) {
        // 使用您在类中定义的 solve_method (已设为 SOLVEPNP_SQPNP)
        // 对于初始PnP，useExtrinsicGuess 应为 false
        initial_pnp_success = cv::solvePnP(objectpoints_3d,
                                           detected_initial_points,
                                           this->cam_obj->camera_matrix_cv,
                                           this->cam_obj->dist_coeffs,
                                           r_vec, // 输出参数
                                           t_vec, // 输出参数
                                           false, // useExtrinsicGuess = false for initial PnP
                                           solve_method); // 使用类成员变量 solve_method

        if (initial_pnp_success) {
            std::cout << "[Marker ID: " << id << " TRACK_INIT_PNP] Initial PnP Succeeded." << std::endl << std::flush;
        } else {
            std::cerr << "[Marker ID: " << id << " TRACK_INIT_PNP] Initial PnP FAILED by solvePnP function." << std::endl << std::flush;
        }
    } else {
        std::cerr << "[Marker ID: " << id << " TRACK_INIT_PNP] Insufficient or mismatched points for initial PnP. "
                  << "Object points: " << objectpoints_3d.size()
                  << ", Detected 2D points: " << detected_initial_points.size()
                  << ". Skipping PnP." << std::endl << std::flush;
        initial_pnp_success = false; // 明确标记失败
    }

    if (!initial_pnp_success) { // 如果初始PnP失败，则停止跟踪并注销标记
        std::cerr << "[Marker ID: " << id << " TRACK] Stopping tracking due to initial PnP failure." << std::endl << std::flush;
        stopTracking();
        markers_manager_ptr->deregisterMarker(this);
        // pnp_running = false; // 如果 track() 是在单独线程中，确保 pnp_running 在退出前被正确设置
        // current_pnp_thread.join(); // 如果有必要等待线程结束
        return; // 线程函数应该返回以终止该标记的跟踪
    }

    // ... (Marker::track() 的后续代码，如投影点、初始化 tracked_blobs 等) ...
    // 您之前的代码：
    std::vector<cv::Point2d> projected;
    cv::projectPoints(objectpoints_3d, r_vec, t_vec, this->cam_obj->camera_matrix_cv, this->cam_obj->dist_coeffs, projected);

    std::vector<cv::Point2d> center_projected;
    std::vector<cv::Point3d> center_3d;
    center_3d.push_back(cv::Point3d(0.0,0.0,0.0));
    cv::projectPoints(center_3d, r_vec, t_vec, this->cam_obj->camera_matrix_cv, this->cam_obj->dist_coeffs, center_projected);
    
    centerpoint_3d = center_3d[0];
    projected_centerpoint = center_projected[0];
    projected_objectpoints = projected;

    tracked_blobs.clear(); // 清空以防万一
    for (size_t i = 0; i < detected_initial_points.size(); ++i){
        tracked_blobs.push_back(std::make_shared<TrackedBlob>(detected_initial_points[i], blob_frequencies[i],detected_at));
    }

    pnp_running = false; // 初始化为 false，将在循环中控制
    new_result = false;  // 初始化

    final_accumulation_time = detected_at + accumulation_period;
    working_buffer = std::make_shared<Deque>();

    // auto out = getOutput(true); // 确保 getOutput 在 r_vec, t_vec 有效时才被正确调用
    // logger_ptr->output_queue->enqueue(out); // 暂时注释或修改 Logger 相关代码

    size_t i_loop_idx = 0; // 重命名循环变量 i，避免与类成员 i 混淆

    while (!tracking_lost) {
        if(i_loop_idx < initial_buffer->size()){ // 使用 i_loop_idx
            buffers.current_batch = initial_buffer->at(i_loop_idx);
            if(buffers.current_batch->first->front()->t >= detected_at){
                new_batch = true;
            }
            i_loop_idx++; // 使用 i_loop_idx
        }
        else{
            new_batch = buffers.getBatch();
        }
        if(new_batch){
            for (auto ev : *buffers.current_batch->first){
                if(!pnp_running.load()){ // 检查 pnp_running 状态
                    pnp_running = true;  // 设置 pnp_running 状态
                    blobs_for_current_pnp.clear();
                    average_distances.clear();
                    for (auto blob : tracked_blobs){
                        blobs_for_current_pnp.push_back(blob->get_output());
                        if((ev->t - blob->last_update) > 8*blob->period){
                            tracking_lost = true;
                            std::cout << "[Marker ID: " << id << " TRACK_LOOP] Tracking lost (inactivity). Event time: " << ev->t
                                      << " vs Blob last_update: " << blob->last_update << " period: " << blob->period
                                      << " freq: " << blob->frequency << std::endl << std::flush;
                            break;
                        }
                        average_distances.push_back(blob->radius);
                    }
                    if(!tracking_lost.load()){
                        // 确保 std::jthread 或 std::thread 的类型与声明一致
                        current_pnp_thread = std::jthread([this](){ // 捕获 this
                            this->solvePnP(); // 调用成员函数
                        });
                    }
                }
                if(tracking_lost.load()) {
                    break;
                }

                for (auto &blob : tracked_blobs) {
                    if(blob->update(ev)){
                        break;
                    }
                }
                current_camera_timestamp = ev->t;
            }
        }
         if (tracking_lost.load()) break; // 如果在内层循环丢失跟踪，也跳出外层while
    }
    std::cout << "[Marker ID: " << id << " TRACK] Exiting track loop. tracking_lost: " << tracking_lost.load() << std::endl << std::flush;
    stopTracking();
    markers_manager_ptr->deregisterMarker(this);
    if (current_pnp_thread.joinable()) { // 确保在线程销毁前 join
        current_pnp_thread.join();
    }
}


Marker::Marker(int idx,
               std::vector<cv::Point3d> points_3d,
               std::vector<std::vector<cv::Point>> contours,
               std::vector<int> frequencies,
               long long detection_time,
               Camera *cam,std::shared_ptr<Logger> logger,
               MarkersManager *markers_manager)
{
    markers_manager_ptr = markers_manager;
    logger_ptr = logger;
    detected_at = detection_time;
    current_camera_timestamp = detection_time;

    blob_frequencies = frequencies;
    auto now_ms = std::chrono::system_clock::now();
    auto epoch = now_ms.time_since_epoch();
    auto value = std::chrono::duration_cast<std::chrono::microseconds>(epoch);

    current_pc_timestamp = value.count();


    id = idx;
    std::vector<cv::Point2d> centers;
    for (unsigned int i=0; i<contours.size(); i++){
        cv::Moments M = cv::moments(contours[i]);
        cv::Point2f center(M.m10/M.m00, M.m01/M.m00);
        centers.push_back(center);
    }
    objectpoints_3d = points_3d;
    detected_initial_points = centers;

    cam_obj = cam;

}

void Marker::solvePnP() {
    std::vector<cv::Point2d> blob_centers;
    std::vector<double> distances;

    // 填充 blob_centers 和 distances (假设这部分逻辑正确)
    for (auto blob_out : blobs_for_current_pnp){ // 使用不同的变量名避免歧义
        blob_centers.emplace_back(cv::Point2d(blob_out.x, blob_out.y));
    }
    for (auto distance : average_distances){
        distances.push_back(distance);
    }

    std::cout << "[Marker ID: " << id << " SOLVE_PNP] Attempting PnP. "
              << "Object points count: " << objectpoints_3d.size()
              << ", Image points (blob_centers) count: " << blob_centers.size()
              << std::endl << std::flush;

    bool pnp_call_success = false; // 用于接收 solvePnP 的返回值
    if (objectpoints_3d.size() == blob_centers.size() && objectpoints_3d.size() >= 3) {
        // 使用类成员 solve_method (已设为 SOLVEPNP_SQPNP)
        // 在跟踪循环中，useExtrinsicGuess 可以为 true，使用上一帧的 r_vec, t_vec 作为初始值
        // 但要确保 r_vec 和 t_vec 在第一次调用或丢失跟踪后是合理的，或者在这些情况下设为false
        bool use_guess = !r_vec.empty() && !t_vec.empty(); // 简单的检查，可以更复杂

        pnp_call_success = cv::solvePnP(objectpoints_3d,
                                        blob_centers,
                                        this->cam_obj->camera_matrix_cv,
                                        this->cam_obj->dist_coeffs,
                                        r_vec, // 输出参数
                                        t_vec, // 输出参数
                                        use_guess, // 根据情况设置
                                        solve_method);

        if (pnp_call_success) {
            std::cout << "[Marker ID: " << id << " SOLVE_PNP] PnP call Succeeded." << std::endl << std::flush;
        } else {
            std::cerr << "[Marker ID: " << id << " SOLVE_PNP] PnP call FAILED by solvePnP function." << std::endl << std::flush;
        }
    } else {
        std::cerr << "[Marker ID: " << id << " SOLVE_PNP] Insufficient or mismatched points for PnP. "
                  << "Object points: " << objectpoints_3d.size()
                  << ", Detected 2D points: " << blob_centers.size()
                  << ". Skipping PnP." << std::endl << std::flush;
        pnp_call_success = false; // 明确标记失败
    }

    if (!pnp_call_success) {
        tracking_lost = true; // 如果PnP失败，标记跟踪丢失
        pnp_running = false;  // 重置 PnP 运行状态
        new_result = false;
        return; // 从 solvePnP 返回
    }

    // 只有当 PnP 成功时才继续后续操作
    cv::Mat projected_tmp;
    cv::projectPoints(objectpoints_3d, r_vec, t_vec, this->cam_obj->camera_matrix_cv, this->cam_obj->dist_coeffs, projected_tmp);
    // 检查 projected_tmp 是否为空以及类型和尺寸是否与 projected_objectpoints 匹配，然后再转换
    if (!projected_tmp.empty() && projected_tmp.type() == CV_64FC2 && projected_tmp.rows == (int)objectpoints_3d.size() && projected_tmp.cols == 1) { // 假设是Nx1的2通道Mat
        projected_tmp.reshape(2, projected_tmp.rows).convertTo(projected_objectpoints, CV_64F); // 假设 projected_objectpoints 是 Nx2 CV_64F
        // 或者如果 projected_objectpoints 是 std::vector<cv::Point2d>
        // projected_objectpoints.clear();
        // for(int i=0; i < projected_tmp.rows; ++i) {
        //    projected_objectpoints.push_back(projected_tmp.at<cv::Point2d>(i));
        // }
    } else if (!projected_tmp.empty() && projected_objectpoints.empty()) { // 如果 projected_objectpoints 是空的 std::vector<cv::Point2d>
         projected_objectpoints.clear();
         for(int i=0; i < projected_tmp.rows; ++i) {
            projected_objectpoints.push_back(projected_tmp.at<cv::Point2d>(i));
         }
    }
     else if (!projected_tmp.empty() && !projected_objectpoints.empty() && projected_tmp.rows == (int)projected_objectpoints.size()) {
        // 如果 projected_objectpoints 是 vector<Point2d> 且大小匹配
        for(int i=0; i < projected_tmp.rows; ++i) {
            projected_objectpoints[i] = projected_tmp.at<cv::Point2d>(i);
        }
    }
    else {
        std::cerr << "[Marker ID: " << id << " SOLVE_PNP] Error in converting projected points or projected_tmp is empty/invalid." << std::endl << std::flush;
        // 可能需要将 tracking_lost 设为 true
    }


    double error_sum = 0.;
    double dist_sum = 0.;
    // 确保 projected_objectpoints 和 blob_centers 大小一致
    if (projected_objectpoints.size() == blob_centers.size() && !blob_centers.empty()) {
        for (size_t i = 0; i < blob_centers.size() ; i++) {
            error_sum += cv::norm(projected_objectpoints[i] - blob_centers[i]);
            if (i < distances.size()) { // 确保 distances 也有足够的元素
                 dist_sum += (2 * distances[i]);
            } else {
                 std::cerr << "[Marker ID: " << id << " SOLVE_PNP] Distances vector too small." << std::endl << std::flush;
                 // 可能需要处理这个错误
            }
        }
        std::cout << "[Marker ID: " << id << " SOLVE_PNP] Reprojection error sum: " << error_sum 
                  << ", Distance sum for threshold: " << dist_sum << std::endl << std::flush;
        if(error_sum > dist_sum && !blob_centers.empty()){ // 仅当有blob时才比较
            std::cout << "[Marker ID: " << id << " SOLVE_PNP] Tracking lost due to high reprojection error." << std::endl << std::flush;
            tracking_lost = true;
        }
    } else {
        std::cerr << "[Marker ID: " << id << " SOLVE_PNP] Mismatch or empty projected_objectpoints/blob_centers for error calculation." << std::endl << std::flush;
        if (!blob_centers.empty()) { // 如果只是大小不匹配但有blobs，可能也算丢失
            tracking_lost = true;
        }
    }


    auto now_ms = std::chrono::system_clock::now();
    auto epoch = now_ms.time_since_epoch();
    auto value = std::chrono::duration_cast<std::chrono::microseconds>(epoch);
    current_pc_timestamp = value.count();

    // 确保 getOutput() 内部对 r_vec 和 t_vec 的访问是安全的
    auto out = getOutput(); // getOutput 现在应该内部检查 r_vec, t_vec
    out->tracker_outputs = blobs_for_current_pnp;
    // logger_ptr->output_queue->enqueue(out); // 暂时注释或修改 Logger 相关代码

    pnp_running = false; // PnP 计算完成
    new_result = true;   // 有新的结果
}