#include "halo_radar.h"

#include <net/if.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sstream>
#include <cstring>
#include <iostream>
#include <unistd.h>
#include <algorithm>
#include <ctime>
#include <sys/socket.h>
#include <stdexcept>

namespace simrad_halo_radar
{
    
bool validInterface(const ifaddrs* i)
{
    return i && i->ifa_addr && i->ifa_addr->sa_family == AF_INET && (i->ifa_flags & IFF_UP) > 0 && (i->ifa_flags & IFF_LOOPBACK) == 0 && (i->ifa_flags & IFF_MULTICAST) > 0;
}

std::vector<uint32_t> getLocalAddresses()
{
    std::vector<uint32_t> ret;
    ifaddrs *addr_list;
    if (!getifaddrs(&addr_list))
    {
        for (ifaddrs * addr = addr_list; addr; addr = addr->ifa_next)
            if(validInterface(addr))
                ret.push_back(((sockaddr_in *)(addr->ifa_addr))->sin_addr.s_addr);
        freeifaddrs(addr_list);
    }
    return ret;
}

std::string ipAddressToString(uint32_t a)
{
    std::stringstream ret;
    const uint8_t *bytes = reinterpret_cast<const uint8_t*>(&a);
    ret << int(bytes[0]) << "." << int(bytes[1]) << "." << int(bytes[2]) << "." << int(bytes[3]);
    return ret.str();
}

uint32_t ipAddressFromString(const std::string &a)
{
    return inet_addr(a.c_str());
}

std::vector<AddressSet> scan()
{
    return scan(getLocalAddresses());
}

std::vector<AddressSet> scan(const std::vector<uint32_t> & addresses)
{
    std::vector<AddressSet> ret;
    for(auto a: addresses)
    {
        std::cerr << "local interface:" << ipAddressToString(a) << std::endl;
        int listen_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if(listen_sock < 0)
        {
            perror("socket");
            continue;
        }
        int one = 1;
        if (setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, (const char *)&one, sizeof(one)))
        {
            perror("reuse");
            close(listen_sock);
            continue;
        }
        timeval timeout;      
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;
        if (setsockopt (listen_sock, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(timeout)) < 0)
        {
            perror("socket set timeout");
            close(listen_sock);
            continue;
        }
        sockaddr_in listenAddress;
        memset(&listenAddress, 0, sizeof(listenAddress));
        listenAddress.sin_family = AF_INET;
        listenAddress.sin_addr.s_addr = htonl(INADDR_ANY);
        listenAddress.sin_port = htons(6878);
        if (bind(listen_sock, (sockaddr *)&listenAddress, sizeof(listenAddress)) < 0)
        {
            perror("bind");
            close(listen_sock);
            continue;
        }
        ip_mreq mreq;
        mreq.imr_interface.s_addr = a;
        in_addr ma;
        uint8_t ma_bytes[] = {236,6,7,5};
        ma.s_addr = *reinterpret_cast<uint32_t *>(ma_bytes);
        mreq.imr_multiaddr = ma;
        if (setsockopt(listen_sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, (const char *)&mreq, sizeof(mreq)))
        {
            perror("multicast add membership");
            close(listen_sock);
            continue;
        }
        int send_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (setsockopt(send_sock, SOL_SOCKET, SO_REUSEADDR, (const char *)&one, sizeof(one)))
        {
            perror("reuse");
            close(listen_sock);
            close(send_sock);
            continue;
        }
        sockaddr_in sendAddress;
        memset(&sendAddress, 0, sizeof(sendAddress));
        sendAddress.sin_family = AF_INET;
        sendAddress.sin_addr.s_addr = a;
        if(bind(send_sock, (sockaddr *)&sendAddress, sizeof(sendAddress)) < 0)
        {
            perror("bind");
            close(listen_sock);
            close(send_sock);
            continue;
        }
        uint16_t data = 0xb101;
        
        sendAddress.sin_addr.s_addr = ma.s_addr;
        sendAddress.sin_port = htons(6878);
        if(sendto(send_sock,&data,sizeof(data),0,(sockaddr*)&sendAddress,sizeof(sendAddress))!=sizeof(data))
        {
            perror("sendto");
            close(listen_sock);
            close(send_sock);
            continue;
        }
        
        int count = 0;
        while(count < 3)
        {
            uint8_t in_data[1024];
            sockaddr_in from_addr;
            unsigned int from_addr_len = sizeof(from_addr);
            int nbytes = recvfrom(listen_sock,in_data,1024,0,(sockaddr*)&from_addr,&from_addr_len);
            if(nbytes > 0)
            {
                std::cerr << nbytes << " bytes" << std::endl;
                std::cerr << "is it " << sizeof(RadarReport_b201) << " bytes and start with b201?" << std::endl;
                RadarReport_b201* b201 =  reinterpret_cast<RadarReport_b201*>(in_data);
                if(nbytes == sizeof(RadarReport_b201) && b201->id == 0xb201)
                {
                    AddressSet asa;
                    asa.label = "halo_a";
                    asa.serial_number = std::string(
                        b201->serialno, strnlen(b201->serialno, sizeof(b201->serialno)));
                    asa.data = b201->addrDataA;
                    asa.send = b201->addrSendA;
                    asa.report = b201->addrReportA;
                    asa.interface = a;
                    ret.push_back(asa);
                    AddressSet asb;
                    asb.label = "halo_b";
                    asb.serial_number = asa.serial_number;
                    asb.data = b201->addrDataB;
                    asb.send = b201->addrSendB;
                    asb.report = b201->addrReportB;
                    asb.interface = a;
                    ret.push_back(asb);
                    break;
                }
            }
            count += 1;
        }
        close(listen_sock);
        close(send_sock);
        if(!ret.empty())
            break;
    }
    
    return ret;
}

std::string AddressSet::str() const
{
    std::stringstream ret;
    ret << label << ": data: " << ipAddressToString(data.address) << ":" << ntohs(data.port) << ", report: " << ipAddressToString(report.address) << ":" << ntohs(report.port) << ", send: " << ipAddressToString(send.address) << ":" << ntohs(send.port) << ", interface: " << ipAddressToString(interface);
    return ret.str();
}

Radar::Radar(AddressSet const &addresses):m_addresses(addresses),m_exitFlag(false)
{
    m_sendSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (m_sendSocket < 0)
        throw std::runtime_error("could not create HALO command socket");
    int one = 1;
    if (setsockopt(m_sendSocket, SOL_SOCKET, SO_REUSEADDR,
                   (const char *)&one, sizeof(one)) < 0)
    {
        close(m_sendSocket);
        m_sendSocket = -1;
        throw std::runtime_error("could not configure HALO command socket");
    }
    
    memset(&m_sendAddress, 0, sizeof(m_sendAddress));
    m_sendAddress.sin_family = AF_INET;
    m_sendAddress.sin_addr.s_addr = addresses.interface;
    if (bind(m_sendSocket, (sockaddr *)&m_sendAddress, sizeof(m_sendAddress)) < 0)
    {
        close(m_sendSocket);
        m_sendSocket = -1;
        throw std::runtime_error("could not bind HALO command socket to the selected interface");
    }
    
    m_sendAddress.sin_addr.s_addr = addresses.send.address;
    m_sendAddress.sin_port = addresses.send.port;
    
    sendHeartbeat();
}

Radar::~Radar()
{
    stopThreads();
    if (m_sendSocket >= 0)
    {
        close(m_sendSocket);
        m_sendSocket = -1;
    }
}

void Radar::stopThreads()
{
    {
        const std::lock_guard<std::mutex> lock(m_exitFlagMutex);
        m_exitFlag = true;
    }
    if (m_dataThread.joinable())
      m_dataThread.join();
    if (m_reportThread.joinable())
      m_reportThread.join();
}

void Radar::startThreads()
{
    m_dataThread = std::thread(&Radar::dataThread,this);
    m_reportThread = std::thread(&Radar::reportThread,this);
}

int Radar::createListenerSocket(uint32_t interface, uint32_t mcast_address, uint16_t port)
{
    int ret = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if(ret < 0)
        return ret;
    int one = 1;
    if (setsockopt(ret, SOL_SOCKET, SO_REUSEADDR, (const char *)&one, sizeof(one)))
    {
        close(ret);
        return -1;
    }
#ifdef SO_TIMESTAMPNS
    // Ask the kernel to attach a CLOCK_REALTIME timestamp to each datagram.
    // dataThread falls back to clock_gettime when this is unavailable.
    setsockopt(ret, SOL_SOCKET, SO_TIMESTAMPNS, (const char *)&one, sizeof(one));
#endif
#ifdef SO_RXQ_OVFL
    // Request a cumulative count of UDP packets dropped by the socket receive
    // queue. This distinguishes local overload from upstream/network loss.
    setsockopt(ret, SOL_SOCKET, SO_RXQ_OVFL, (const char *)&one, sizeof(one));
#endif
    timeval timeout;      
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;
    if (setsockopt (ret, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(timeout)) < 0)
    {
        close(ret);
        return -1;
    }
    sockaddr_in listenAddress;
    memset(&listenAddress, 0, sizeof(listenAddress));
    listenAddress.sin_family = AF_INET;
    listenAddress.sin_addr.s_addr = htonl(INADDR_ANY);
    listenAddress.sin_port = port;
    if (bind(ret, (sockaddr *)&listenAddress, sizeof(listenAddress)) < 0)
    {
        close(ret);
        return -1;
    }
    ip_mreq mreq;
    mreq.imr_interface.s_addr = interface;
    mreq.imr_multiaddr.s_addr = mcast_address;
    if (setsockopt(ret, IPPROTO_IP, IP_ADD_MEMBERSHIP, (const char *)&mreq, sizeof(mreq)))
    {
        close(ret);
        return -1;
    }
    return ret;
}


void Radar::dataThread()
{
    const int data_socket = createListenerSocket(
        m_addresses.interface, m_addresses.data.address, m_addresses.data.port);
    if (data_socket < 0)
    {
        perror("data socket");
        return;
    }

    uint8_t in_data[65535];
    while (true)
    {
        {
            const std::lock_guard<std::mutex> lock(m_exitFlagMutex);
            if (m_exitFlag)
                break;
        }

        sockaddr_in from_addr{};
        iovec iov{};
        iov.iov_base = in_data;
        iov.iov_len = sizeof(in_data);
        char control[CMSG_SPACE(sizeof(timespec)) + CMSG_SPACE(sizeof(uint32_t))]{};
        msghdr message{};
        message.msg_name = &from_addr;
        message.msg_namelen = sizeof(from_addr);
        message.msg_iov = &iov;
        message.msg_iovlen = 1;
        message.msg_control = control;
        message.msg_controllen = sizeof(control);

        const ssize_t nbytes = recvmsg(data_socket, &message, 0);
        if (nbytes <= 0)
            continue;

        timespec arrival{};
        clock_gettime(CLOCK_REALTIME, &arrival);
        DatagramMetadata metadata;
        for (cmsghdr *cmsg = CMSG_FIRSTHDR(&message); cmsg != nullptr;
             cmsg = CMSG_NXTHDR(&message, cmsg))
        {
#ifdef SCM_TIMESTAMPNS
            if (cmsg->cmsg_level == SOL_SOCKET && cmsg->cmsg_type == SCM_TIMESTAMPNS &&
                cmsg->cmsg_len >= CMSG_LEN(sizeof(timespec)))
            {
                std::memcpy(&arrival, CMSG_DATA(cmsg), sizeof(arrival));
                continue;
            }
#endif
#ifdef SO_RXQ_OVFL
            if (cmsg->cmsg_level == SOL_SOCKET && cmsg->cmsg_type == SO_RXQ_OVFL &&
                cmsg->cmsg_len >= CMSG_LEN(sizeof(uint32_t)))
            {
                std::memcpy(
                    &metadata.kernel_drop_count, CMSG_DATA(cmsg), sizeof(uint32_t));
                metadata.has_kernel_drop_count = true;
            }
#endif
        }

        metadata.arrival_time_ns =
            static_cast<int64_t>(arrival.tv_sec) * 1000000000LL + arrival.tv_nsec;
        metadata.source_address = ipAddressToString(from_addr.sin_addr.s_addr);
        metadata.source_port = ntohs(from_addr.sin_port);
        metadata.socket_message_flags = static_cast<uint32_t>(message.msg_flags);
        metadata.datagram_truncated = (message.msg_flags & MSG_TRUNC) != 0;

        for (const auto &sector : m_sectorDecoder.decode(
                 in_data, static_cast<size_t>(nbytes), metadata))
        {
            processData(sector);
        }
    }
    close(data_socket);
}

void Radar::reportThread()
{
    int report_socket = createListenerSocket(m_addresses.interface,
    m_addresses.report.address, m_addresses.report.port);
    if(report_socket < 0)
    {
        perror("report socket");
        return;
    }

    uint8_t in_data[65535];
    while(true)
    {
        {
            const std::lock_guard<std::mutex> lock(m_exitFlagMutex);
            if(m_exitFlag)
                break;
        }
        sockaddr_in from_addr;
        unsigned int from_addr_len = sizeof(from_addr);
        const ssize_t nbytes = recvfrom(
            report_socket, in_data, sizeof(in_data), 0,
            reinterpret_cast<sockaddr *>(&from_addr), &from_addr_len);
        if(nbytes > 0)
        {
            if(nbytes >= 2)
            {
                std::map<std::string,std::string> new_state;
                uint16_t id = *reinterpret_cast<uint16_t*>(in_data);

                switch(id)
                {
                    case 0xc401:
                        if (nbytes < 3)
                            break;
                        switch(in_data[2])
                        {
                            case 1:
                                new_state["status"] = "standby";
                                break;
                            case 2:
                                new_state["status"] = "transmit";
                                break;
                            case 5:
                                new_state["status"] = "spinning_up";
                                break;
                            default:
                                new_state["status"] = "unknown";
                        }
                        break;
                    case 0xc402:
                    {
                        RadarReport_c402 *c402 = reinterpret_cast<RadarReport_c402*>(in_data);
                        if(nbytes >= static_cast<ssize_t>(sizeof(RadarReport_c402)))
                        {
                            new_state["range"] = std::to_string(c402->range/10);
                            
                            switch(c402->mode)
                            {
                                case 0:
                                    new_state["mode"] = "custom";
                                    break;
                                case 1:
                                    new_state["mode"] = "harbor";
                                    break;
                                case 2:
                                    new_state["mode"] = "offshore";
                                    break;
                                case 4:
                                    new_state["mode"] = "weather";
                                    break;
                                case 5:
                                    new_state["mode"] = "bird";
                                    break;
                                default:
                                    new_state["mode"] = "unknown";
                            }
                            
                            new_state["gain"] = std::to_string(c402->gain*100/255.0);
                            if(c402->gain_auto)
                                new_state["gain_mode"] = "auto";
                            else
                                new_state["gain_mode"] = "manual";
                            
                            new_state["sea_clutter"] = std::to_string(c402->sea_clutter*100/255.0);
                            if(c402->sea_clutter_auto)
                                new_state["sea_clutter_mode"] = "auto";
                            else
                                new_state["sea_clutter_mode"] = "manual";
                            
                            new_state["rain_clutter"] = std::to_string(c402->rain_clutter*100/255.0);
                            
                            switch(c402->interference_rejection)
                            {
                                case 0:
                                    new_state["interference_rejection"] = "off";
                                    break;
                                case 1:
                                    new_state["interference_rejection"] = "low";
                                    break;
                                case 2:
                                    new_state["interference_rejection"] = "medium";
                                    break;
                                case 3:
                                    new_state["interference_rejection"] = "high";
                                    break;
                                default:
                                    new_state["interference_rejection"] = "unknown";
                            }
                            
                            switch(c402->target_expansion)
                            {
                                case 0:
                                    new_state["target_expansion"] = "off";
                                    break;
                                case 1:
                                    new_state["target_expansion"] = "low";
                                    break;
                                case 2:
                                    new_state["target_expansion"] = "medium";
                                    break;
                                case 3:
                                    new_state["target_expansion"] = "high";
                                    break;
                                default:
                                    new_state["target_expansion"] = "unknown";
                            }
                        }
                        break;
                    }
                    case 0xc403:
                        // not sure
                        break;
                    case 0xc404:
                    {
                        RadarReport_c404 *c404 = reinterpret_cast<RadarReport_c404*>(in_data);
                        if(nbytes >= static_cast<ssize_t>(sizeof(RadarReport_c404)))
                        {
                            new_state["bearing_alignment"] = std::to_string(c404->bearing_alignment/10.0);
                            new_state["antenna_height"] = std::to_string(c404->antenna_height/1000.0);
                            switch(c404->lights)
                            {
                                case 0:
                                    new_state["lights"] = "off";
                                    break;
                                case 1:
                                    new_state["lights"] = "low";
                                    break;
                                case 2:
                                    new_state["lights"] = "medium";
                                    break;
                                case 3:
                                    new_state["lights"] = "high";
                                    break;
                                default:
                                    new_state["lights"] = "unknown";
                            }
                        }
                        break;
                    }
                    case 0xc406:
                        // not sure
                        break;
                    case 0xc408:
                    {
                        RadarReport_c408 *c408 = reinterpret_cast<RadarReport_c408*>(in_data);
                        if(nbytes >= static_cast<ssize_t>(sizeof(RadarReport_c408)))
                        {
                            switch(c408->sea_state)
                            {
                                case 0:
                                    new_state["sea_state"] = "calm";
                                    break;
                                case 1:
                                    new_state["sea_state"] = "moderate";
                                    break;
                                case 2:
                                    new_state["sea_state"] = "rough";
                                    break;
                                default:
                                    new_state["sea_state"] = "unknown";
                            }
                            
                            switch(c408->scan_speed)
                            {
                                case 0:
                                    new_state["scan_speed"] = "off";
                                    break;
                                case 1:
                                    new_state["scan_speed"] = "medium";
                                    break;
                                case 3:
                                    new_state["scan_speed"] = "high";
                                    break;
                                default:
                                    new_state["scan_speed"] = "default";
                            }
                            
                            if(c408->sls_auto)
                                new_state["sidelobe_suppression_mode"] = "auto";
                            else
                                new_state["sidelobe_suppression_mode"] = "manual";
                            new_state["sidelobe_suppression"] = std::to_string(c408->side_lobe_suppression*100/255.0);
                            
                            switch(c408->noise_rejection)
                            {
                                case 0:
                                    new_state["noise_rejection"] = "off";
                                    break;
                                case 1:
                                    new_state["noise_rejection"] = "low";
                                    break;
                                case 2:
                                    new_state["noise_rejection"] = "medium";
                                    break;
                                case 3:
                                    new_state["noise_rejection"] = "high";
                                    break;
                                default:
                                    new_state["noise_rejection"] = "unknown";
                            }
                            
                            switch(c408->target_separation)
                            {
                                case 0:
                                    new_state["target_separation"] = "off";
                                    break;
                                case 1:
                                    new_state["target_separation"] = "low";
                                    break;
                                case 2:
                                    new_state["target_separation"] = "medium";
                                    break;
                                case 3:
                                    new_state["target_separation"] = "high";
                                    break;
                                default:
                                    new_state["target_separation"] = "unknown";
                            }
                            
                            new_state["auto_sea_clutter_nudge"] = std::to_string(c408->auto_sea_clutter_nudge);
                            
                            switch(c408->doppler_state)
                            {
                                case 0:
                                    new_state["doppler_mode"] = "off";
                                    break;
                                case 1:
                                    new_state["doppler_mode"] = "normal";
                                    break;
                                case 2:
                                    new_state["doppler_mode"] = "approaching_only";
                                    break;
                                default:
                                    new_state["doppler_mode"] = "unknown";
                            }
                            
                            new_state["doppler_speed"] = std::to_string(c408->doppler_speed/100.0);
                        }
                        break;
                    }
                    case 0xc409:
                        break;
                    case 0xc40a:
                        break;
                    case 0xc611:
                        // heartbeat
                        break;
                    default:
                        std::cerr << m_addresses.label << " " << nbytes << " bytes of report data, ";
                        std::cerr << "id: " << std::showbase << std::hex << id << std::noshowbase << std::dec << std::endl;                        
                }
                
                bool state_updated = false;
                for(auto kv: new_state)
                {
                    if(m_state.find(kv.first) == m_state.end() || m_state[kv.first] != kv.second)
                    {
                       m_state[kv.first] = kv.second;
                       state_updated = true;
                    }
                }
                
                if(state_updated)
                    this->stateUpdated();
            }
        }
    }
    close(report_socket);
}

void Radar::sendCommand(const uint8_t data[], int size)
{
    if (m_sendSocket >= 0)
        sendto(m_sendSocket, data, size, 0,
               reinterpret_cast<sockaddr *>(&m_sendAddress), sizeof(m_sendAddress));
}

void Radar::sendHeartbeat()
{
    
    uint8_t data1[] = {0xa0,0xc1};
    sendCommand(data1,2);

    uint8_t data2[] = {0x03,0xc2};
    sendCommand(data2,2);

    uint8_t data3[] = {0x04,0xc2};
    sendCommand(data3,2);

    uint8_t data4[] = {0x05,0xc2};
    sendCommand(data4,2);
    
    m_lastHeartbeat = std::chrono::system_clock::now();
}

bool Radar::checkHeartbeat()
{
    auto elapsed = std::chrono::system_clock::now() - m_lastHeartbeat;
    if(elapsed > std::chrono::seconds(1))
    {
        sendHeartbeat();
        return true;
    }
    return false;
}

void Radar::sendCommand(std::string const &key, std::string const &value)
{
    if(key == "status")
    {
        if(value == "transmit")
        {
            uint8_t data1[] = {0x00,0xc1,0x01};
            sendCommand(data1,3);
            uint8_t data2[] = {0x01,0xc1,0x01};
            sendCommand(data2,3);
        }
        else if(value == "standby")
        {
            uint8_t data1[] = {0x00,0xc1,0x01};
            sendCommand(data1,3);
            uint8_t data2[] = {0x01,0xc1,0x00};
            sendCommand(data2,3);
        }
    }
    if(key == "range")
    {
        RangeCmd cmd;
        cmd.range = std::stof(value)*10;
        sendCommand(cmd);
    }
    if(key == "bearing_alignment")
    {
        BearingAlignmentCmd cmd;
        cmd.bearing_alignment = std::stof(value)*10;
        sendCommand(cmd);
    }
    if(key == "gain")
    {
        GainCmd cmd;
        if (value == "auto")
            cmd.gain_auto = 1;
        else
            cmd.gain = std::stof(value)*255/100;
        sendCommand(cmd);
    }
    if(key == "sea_clutter")
    {
        SeaClutterCmd cmd;
        if (value == "auto")
            cmd.sea_clutter_auto = 1;
        else
            cmd.sea_clutter = std::stof(value)*255/100;
        sendCommand(cmd);
    }
    if(key == "rain_clutter")
    {
        RainClutterCmd cmd;
        cmd.rain_clutter = std::stof(value)*255/100;
        sendCommand(cmd);
    }
    if(key == "sidelobe_suppression")
    {
        SidelobeSuppressionCmd cmd;
        if (value == "auto")
            cmd.sls_auto = 1;
        else
            cmd.sidelobe_suppression = std::stof(value)*255/100;
        sendCommand(cmd);
    }
    std::map<std::string,uint8_t> lmhMap;
    lmhMap["off"] = 0;
    lmhMap["low"] = 1;
    lmhMap["medium"] = 2;
    lmhMap["high"] = 3;
    if(key == "interference_rejection")
        sendCommand(EnumCmd(0xc108,lmhMap[value]));
    if(key == "sea_state")
    {
        EnumCmd cmd(0xc10b,0);
        if(value == "moderate")
            cmd.value = 1;
        if(value == "rough")
            cmd.value = 2;
        sendCommand(cmd);
    }
    if(key == "scan_speed")
    {
        EnumCmd cmd(0xc10f,0);
        if(value == "medium")
            cmd.value = 1;
        if(value == "high")
            cmd.value = 3;
        sendCommand(cmd);
    }
    if(key == "mode")
    {
        EnumCmd cmd(0xc110,0);
        if(value == "harbor")
            cmd.value = 1;
        if(value == "offshore")
            cmd.value = 2;
        if(value == "weather")
            cmd.value = 4;
        if(value == "bird")
            cmd.value = 5;
        sendCommand(cmd);
    }
    if(key == "auto_sea_clutter_nudge")
        sendCommand(AutoSeaClutterNudgeCmd(std::stof(value)));
    if(key == "target_expansion")
        sendCommand(EnumCmd(0xc112,lmhMap[value]));
    if(key == "noise_rejection")
        sendCommand(EnumCmd(0xc121,lmhMap[value]));
    if(key == "target_separation")
        sendCommand(EnumCmd(0xc122,lmhMap[value]));
    if(key == "doppler_mode")
    {
        EnumCmd cmd(0xc123,0);
        if(value == "normal")
            cmd.value = 1;
        if(value == "approaching_only")
            cmd.value = 2;
        sendCommand(cmd);
    }
    if(key == "doppler_speed")
        sendCommand(DopplerSpeedCmd(std::stof(value)*100));
    if(key == "antenna_height")
        sendCommand(AntennaHeightCmd(std::stof(value)*1000));
    if(key == "lights")
        sendCommand(EnumCmd(0xc131,lmhMap[value]));
    
    
    
}

HeadingSender::HeadingSender(uint32_t bindAddress)
{
    m_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    int one = 1;
    if (setsockopt(m_socket, SOL_SOCKET, SO_REUSEADDR, (const char *)&one, sizeof(one)))
    {
        perror("HeadingSender reuse");
        close(m_socket);
        m_socket = 0;
        return;
    }

    sockaddr_in address;
    memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = bindAddress;

    if(bind(m_socket, (sockaddr *)&address, sizeof(address)) < 0)
    {
        perror("HeadingSender bind");
        close(m_socket);
        m_socket = 0;
        return;
    }

    memset(&m_sendAddress, 0, sizeof(m_sendAddress));
    m_sendAddress.sin_family = AF_INET;
    uint8_t address_bytes[] = {239,238,55,73};
    m_sendAddress.sin_addr.s_addr = *reinterpret_cast<uint32_t*>(address_bytes);
    uint16_t port=7527;
    m_sendAddress.sin_port = htons(port);

    m_senderThread = std::thread(&HeadingSender::senderThread,this);
}

HeadingSender::~HeadingSender()
{
    {
        const std::lock_guard<std::mutex> lock(m_exitFlagMutex);
        m_exitFlag = true;
    }
    m_senderThread.join();
}

void HeadingSender::senderThread()
{
    while(true)
    {
        {
            const std::lock_guard<std::mutex> lock(m_exitFlagMutex);
            if(m_exitFlag)
                break;
        }   
        
        auto now = std::chrono::system_clock::now();

        if (now-m_lastHeadingSent > m_headingSendInterval)
        {
            m_counter++;
            m_headingPacket.counter = m_counter;
            m_headingPacket.epoch = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
            {
                const std::lock_guard<std::mutex> lock(m_headingMutex);
                m_headingPacket.heading = (uint16_t)(m_heading * 63488.0 / 360.0);
            }
            sendto(m_socket, &m_headingPacket, sizeof(m_headingPacket), 0, (sockaddr*)&m_sendAddress, sizeof(m_sendAddress));
            m_lastHeadingSent = now;
        }
        if (now-m_lastMysterySent > m_mysterySendInterval)
        {
            m_counter++;
            m_mysteryPacket.counter = m_counter;
            m_mysteryPacket.epoch = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
            m_mysteryPacket.mystery1 = 0;
            m_mysteryPacket.mystery2 = 0;
            sendto(m_socket, &m_mysteryPacket, sizeof(m_mysteryPacket), 0, (sockaddr*)&m_sendAddress, sizeof(m_sendAddress));
            m_lastMysterySent = now;
        }
        auto sleepTime = min(m_lastHeadingSent+m_headingSendInterval-now, m_lastMysterySent+m_mysterySendInterval-now);
        std::this_thread::sleep_for(sleepTime);
        
    }
}

void HeadingSender::setHeading(double heading)
{
    const std::lock_guard<std::mutex> lock(m_headingMutex);
    m_heading = heading; 
}

} // namespace halo_radar
