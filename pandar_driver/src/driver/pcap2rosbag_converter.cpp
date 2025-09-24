#include <iostream>
#include <pcap.h>

#include <pandar_msgs/PandarPacket.h>
#include <pandar_msgs/PandarScan.h>
#include <rosbag/bag.h>

class PacketGenerator {
private:
    pcap_t* pcap_pointer_;
    std::string output_bag_path_;
    size_t data_port_;
    size_t azimuth_index_;
    std::function<bool(size_t)> is_valid_packet_;
    std::string model_;
public:
    PacketGenerator(const std::string& pcap_path, const std::string& model, size_t data_port, const std::string& output_bag) {
        if(pcap_path.empty()) {
            std::cerr << "pcap file path is empty" << std::endl;
            exit(1);
        }
        if (model.empty()) {
            std::cerr << "Sensor moodel is empty" << std::endl;
            exit(1);
        }
        if (output_bag.empty()) {
            std::cerr << "Output bag is empty" << std::endl;
            exit(1);
        }
        output_bag_path_ = output_bag;
        model_ = model;
        if (model_ == "Pandar40P" || model_ == "Pandar40M") {
            azimuth_index_ = 2;  // 2 + 124 * [0-9]
            is_valid_packet_ = [](size_t packet_size) { return (packet_size == 1262 || packet_size == 1266); };
        }
        else if (model_ == "PandarQT") {
            azimuth_index_ = 12;  // 12 + 258 * [0-3]
            is_valid_packet_ = [](size_t packet_size) { return (packet_size == 1072); };
        }
        else  if (model_ == "PandarXT-32") {
            azimuth_index_ = 12;  // 12 + 130 * [0-7]
            is_valid_packet_ = [](size_t packet_size) { return (packet_size == 1080); };
        }
        else if (model_ == "Pandar64") {
            azimuth_index_ = 8;  // 8 + 192 * [0-5]
            is_valid_packet_ = [](size_t packet_size) { return (packet_size == 1194 || packet_size == 1198); };
        }
        else if (model_ == "Pandar128E4X") {
            azimuth_index_ = 12;  // 12 + 386 * [0-1]
            is_valid_packet_ = [](size_t packet_size) { return (packet_size == 1117 || packet_size == 861); };
        }
        else if (model_ == "PandarXTM") {
            azimuth_index_ = 12;  // 12 + 130 * [0-7]
            is_valid_packet_ = [](size_t packet_size) { return (packet_size == 820); };
        }
        else if (model_ == "PandarQT128") {
            azimuth_index_ = 12;  // 12 + 512 * [0-1]
            is_valid_packet_ = [](size_t packet_size) { return (packet_size == 1127); };
        }
        else {
            ROS_ERROR("Invalid model name");
        }

        pcap_pointer_ = pcap_open_offline(pcap_path.c_str(), nullptr);
        if (pcap_pointer_ == nullptr) {
            std::cerr << "Error opening pcap file:" << pcap_path << std::endl;
            exit(1);
        }
    }

    void process() {

        rosbag::Bag bag(output_bag_path_, rosbag::bagmode::Write);
        //bag.write("pandar_packets", ros::Time::now(), generator.pcap_pointer_);
        pandar_msgs::PandarScanPtr pandar_scan_ptr(new pandar_msgs::PandarScan);
        pandar_scan_ptr->header.frame_id = "pandar";
        struct pcap_pkthdr pcap_header;
        const u_char* pcap_packet;
        const uint8_t PKT_HEADER_SIZE = 42;
        int current_phase = 0;
        int prev_phase = 0;
        while ((pcap_packet = pcap_next(pcap_pointer_, &pcap_header)) != nullptr) {
            ros::Time ros_time;
            ros_time.sec = pcap_header.ts.tv_sec;
            ros_time.nsec = pcap_header.ts.tv_usec * 1000;
            //if (pcap_header.len >= PKT_HEADER_SIZE) 
            {
                auto data_size = pcap_header.len - PKT_HEADER_SIZE;
                if (is_valid_packet_(data_size)) {
                    pandar_msgs::PandarPacket pandar_packet;
                    pandar_packet.stamp = ros_time;
                    memcpy(&pandar_packet.data, (pcap_packet+PKT_HEADER_SIZE), data_size);
                    pandar_packet.size = data_size;
                    current_phase = (pandar_packet.data[azimuth_index_] & 0xff) | ((pandar_packet.data[azimuth_index_ + 1] & 0xff) << 8);
                    if (current_phase > prev_phase) {
                        pandar_scan_ptr->packets.push_back(pandar_packet);
                        prev_phase = current_phase;
                    }
                    else {
                        prev_phase = current_phase;
                        pandar_scan_ptr->header.stamp = pandar_scan_ptr->packets.front().stamp;
                        bag.write("pandar_packets", ros_time, pandar_scan_ptr);
                        pandar_scan_ptr->packets.clear();
                        pandar_scan_ptr->packets.push_back(pandar_packet);
                    }
                }//end valid packet
            }//end valid length
        } //end while
    }//end process

    ~PacketGenerator() {
        if (pcap_pointer_ != nullptr)
            pcap_close(pcap_pointer_);
    }
};




int main(int argc, char *argv[]) {
    if (argc < 5) {
        std::cerr << "Usage: pcap_reader <pcap_file> <sensor_model> <udp_data_port> <output_rosbag>" << std::endl;
        std::cerr << "sensor_model: Pandar40P, Pandar64, PandarQT, Pandar128E4X, PandarXT-32, PandarXTM, PandarQT128" << std::endl;
        exit(1);
    }
    ros::Time::init();
    PacketGenerator generator(argv[1], argv[2], atoi(argv[3]), argv[4]);
    generator.process();

    return 0;
}