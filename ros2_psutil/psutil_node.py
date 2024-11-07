import psutil
import time
import rclpy
from rclpy.exceptions import ParameterNotDeclaredException
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from rcl_interfaces.msg import Parameter, ParameterDescriptor, ParameterType, ParameterValue
from ros2_psutil_msgs.msg import *
from std_msgs.msg import Header
from builtin_interfaces.msg import Time

class PsutilNode(Node):

    def __init__(self):
        super().__init__('psutil_node')

        update_frequency_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description='Update frequency to publish in Hz.')

        publish_individual_topics_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_BOOL,
            description='Publish individual topics instead of unified psutil topic.')

        publish_memory_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_BOOL,
            description='Publish memory.')

        publish_network_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_BOOL,
            description='Publish network.')

        publish_network_address_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_BOOL,
            description='Publish network address.')

        publish_network_state_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_BOOL,
            description='Publish network state.')

        publish_network_stats_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_BOOL,
            description='Publish network stats.')

        publish_temperature_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_BOOL,
            description='Publish temperature.')

        publish_processor_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_BOOL,
            description='Publish processor.')

        publish_processor_percent_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_BOOL,
            description='Publish processor percent.')

        publish_processor_frequency_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_BOOL,
            description='Publish processor frequency.')

        allow_network_nic_array_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING_ARRAY,
            description='Array of network NICs to return, to return all leave empty.')

        allow_network_address_family_array_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_INTEGER_ARRAY,
            description='''Int array of network address families to return, valid values are: 
            AF_INET: 2, 
            AF_INET6: 10, 
            AF_PACKET: 17, 
            to return all leave empty.''')

        allow_device_temperatures_array_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING_ARRAY,
            description='Array of devices to return temperatures for, to return all leave empty.')

        allow_temperature_name_array_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING_ARRAY,
            description='Array of temperture names on a device to return, to return all leave empty.')
        
        self.declare_parameter("update_frequency", 1.0,
            update_frequency_descriptor)

        self.declare_parameter("individual_topics", False, 
            publish_individual_topics_descriptor)

        self.declare_parameter("mem", False, 
            publish_memory_descriptor)
        
        self.declare_parameter("net", False, 
            publish_network_descriptor)

        self.declare_parameter("net_addr", False, 
            publish_network_address_descriptor)
        
        self.declare_parameter("net_state", False, 
            publish_network_state_descriptor)
        
        self.declare_parameter("net_stats", False, 
            publish_network_stats_descriptor)
        
        self.declare_parameter("temp", False, 
            publish_temperature_descriptor)
        
        self.declare_parameter("proc", False, 
            publish_processor_descriptor)
        
        self.declare_parameter("proc_per", False, 
            publish_processor_percent_descriptor)

        self.declare_parameter("proc_freq", False, 
            publish_processor_frequency_descriptor)

        self.declare_parameter("net_nic_match", [""],
            allow_network_nic_array_descriptor)

        self.declare_parameter("net_af_match", [int(-9999)],
            allow_network_address_family_array_descriptor)

        self.declare_parameter("dev_temp_match", [""],
            allow_device_temperatures_array_descriptor)

        self.declare_parameter("temp_name_match", [""],
            allow_temperature_name_array_descriptor)

        self.UpdateFrequency = self.get_parameter("update_frequency").value

        self.PublishIndividualTopics = self.get_parameter("individual_topics").value

        self.PublishMemory = self.get_parameter("mem").value

        self.PublishNetwork = self.get_parameter("net").value

        self.PublishNetworkAddress = self.get_parameter("net_addr").value

        self.PublishNetworkState = self.get_parameter("net_state").value

        self.PublishNetworkStats = self.get_parameter("net_stats").value

        self.PublishTemperature = self.get_parameter("temp").value

        self.PublishProcessor = self.get_parameter("proc").value

        self.PublishProcessorPercent = self.get_parameter("proc_per").value

        self.PublishProcessorFrequency = self.get_parameter("proc_freq").value

        self.NetworkAddressFamilyMatch = self.get_parameter_or("net_af_match",
            rclpy.parameter.Parameter(
                "net_af_match",
                type_=rclpy.parameter.Parameter.Type.INTEGER_ARRAY,
                value=[])).value

        self.NetworkNICMatch = self.get_parameter_or("net_nic_match",
            rclpy.parameter.Parameter(
                "net_nic_match",
                type_=rclpy.parameter.Parameter.Type.STRING_ARRAY,
                value=[])).value

        self.TemperatureDeviceMatch = self.get_parameter_or("dev_temp_match",
            rclpy.parameter.Parameter(
                "dev_temp_match",
                type_=rclpy.parameter.Parameter.Type.STRING_ARRAY,
                value=[])).value

        self.TemperatureNameMatch = self.get_parameter_or("temp_name_match",
            rclpy.parameter.Parameter(
                "temp_name_match",
                type_=rclpy.parameter.Parameter.Type.STRING_ARRAY,
                value=[])).value

        if len(self.NetworkAddressFamilyMatch) == 1:
            if self.NetworkAddressFamilyMatch[0] == -9999:
                self.NetworkAddressFamilyMatch = []

        if len(self.NetworkNICMatch) == 1:
            if self.NetworkNICMatch[0] == "":
                self.NetworkNICMatch = []

        if len(self.TemperatureDeviceMatch) == 1:
            if self.TemperatureDeviceMatch[0] == "":
                self.TemperatureDeviceMatch = []

        if len(self.TemperatureNameMatch) == 1:
            if self.TemperatureNameMatch[0] == "":
                self.TemperatureNameMatch = []

        if self.PublishIndividualTopics:
            if self.PublishMemory:
                self.MemoryTopic = "/psutil/memory"
                self.MemoryPub = self.create_publisher(PsutilMemoryStamped,'{:s}'.format(self.MemoryTopic), 1)
            
            if self.PublishNetwork:
                self.NetworkTopic = "/psutil/network"
                self.NetworkPub = self.create_publisher(PsutilNetworkStamped,'{:s}'.format(self.NetworkTopic), 1)

            if self.PublishProcessor:
                self.ProcessorTopic = "/psutil/processor"
                self.ProcessorPub = self.create_publisher(PsutilProcessorStamped,'{:s}'.format(self.ProcessorTopic), 1)

            if self.PublishTemperature:
                self.TemperatureTopic = "/psutil/temperature"
                self.TemperaturePub = self.create_publisher(PsutilTemperatureStamped,'{:s}'.format(self.TemperatureTopic), 1)

        else:
            self.PsutilTopic = "/psutil/system"
            self.PsutilPub = self.create_publisher(Psutil,'{:s}'.format(self.PsutilTopic), 1)
        
        self.boot_time = psutil.boot_time()
        self.logical_cores = int(psutil.cpu_count())
        self.physical_cores = int(psutil.cpu_count(logical=False))
        psutil.net_io_counters.cache_clear()
        self.timer = self.create_timer(1.0/self.UpdateFrequency, self.timerCallback)

    def timerCallback(self):
        
        # Common
        msgHeader=Header()
        msgHeader.stamp = self.get_clock().now().to_msg()
        if not self.PublishIndividualTopics:
            msg=Psutil()

        # Memory
        if self.PublishMemory:
            memory = psutil.virtual_memory()
            msgM = PsutilMemory()
            msgM.total = int(memory.total or 0)
            msgM.available = int(memory.available or 0)
            msgM.percent = float(memory.percent or 0)
            msgM.used = int(memory.used or 0)
            msgM.free = int(memory.free or 0)
            msgM.active = int(memory.active or 0)
            msgM.inactive = int(memory.inactive or 0)
            msgM.buffers = int(memory.buffers or 0)
            msgM.cached = int(memory.cached or 0)
            msgM.shared = int(memory.shared or 0)
            msgM.slab = int(memory.slab or 0)
            if self.PublishIndividualTopics:
                msgMStamp=PsutilMemoryStamped()
                msgMStamp.header=msgHeader
                msgMStamp.memory = msgM
                self.MemoryPub.publish(msgMStamp)
            else:
                msg.memory = msgM
        
        if self.PublishNetwork:
            if self.PublishIndividualTopics:
                    msgNStamp=PsutilNetworkStamped()
            net_stats = psutil.net_io_counters(pernic=True, nowrap=True)
            net_state = psutil.net_if_stats()
            net_addr = psutil.net_if_addrs()
            if len(self.NetworkNICMatch):
                nic_names = sorted((set(net_state.keys()) | set(net_stats.keys()) | set(net_addr.keys())) & set(self.NetworkNICMatch))
            else:
                nic_names = sorted(set(net_state.keys()) | set(net_stats.keys()) | set(net_addr.keys()))
            for nic in nic_names:
                msgN = PsutilNetworkDevice()
                msgN.nic_name = str(nic)
                if self.PublishNetworkAddress and (nic in net_addr):
                    for net_fam in range(len(net_addr[nic])):
                        NetworkAddressFamily=int(net_addr[nic][net_fam].family or 0)
                        if ((NetworkAddressFamily in self.NetworkAddressFamilyMatch) or
                            (not len(self.NetworkAddressFamilyMatch))):
                            msgNA = PsutilNetworkAddress()
                            msgNA.family = NetworkAddressFamily
                            msgNA.address = str(net_addr[nic][net_fam].address)
                            msgNA.netmask = str(net_addr[nic][net_fam].netmask)
                            msgNA.broadcast = str(net_addr[nic][net_fam].broadcast)
                            msgNA.ptp = str(net_addr[nic][net_fam].ptp)
                            msgN.network_address.append(msgNA)
                
                if self.PublishNetworkState and (nic in net_state):
                    msgNS = PsutilNetworkState()
                    msgNS.isup = bool(net_state[nic].isup)
                    msgNS.duplex = int(net_state[nic].duplex or 0)
                    msgNS.speed = int(net_state[nic].speed or 0)
                    msgNS.mtu = int(net_state[nic].mtu or 0)
                    msgNS.flags = str(net_state[nic].flags)
                    msgN.network_state = msgNS
                
                if self.PublishNetworkStats and (nic in net_stats):
                    msgNSS = PsutilNetworkStats()
                    msgNSS.bytes_sent = int(net_stats[nic].bytes_sent or 0)
                    msgNSS.bytes_recv = int(net_stats[nic].bytes_recv or 0)
                    msgNSS.packets_sent = int(net_stats[nic].packets_sent or 0)
                    msgNSS.packets_recv = int(net_stats[nic].packets_recv or 0)
                    msgNSS.errin = int(net_stats[nic].errin or 0)
                    msgNSS.errout = int(net_stats[nic].errout or 0)
                    msgNSS.dropin = int(net_stats[nic].dropin or 0)
                    msgNSS.dropout = int(net_stats[nic].dropout or 0)
                    msgN.network_stats = msgNSS
                
                if self.PublishIndividualTopics:
                    msgNStamp.networks.append(msgN)
                else:
                    msg.networks.append(msgN)

            if self.PublishIndividualTopics:
                msgNStamp.header=msgHeader
                self.NetworkPub.publish(msgNStamp)
        
        # Temperature
        if self.PublishTemperature:
            temps = psutil.sensors_temperatures(fahrenheit=False)
            if len(self.TemperatureDeviceMatch):
                device_names = sorted(set(temps.keys()) & set(self.TemperatureDeviceMatch))
            else:
                device_names = sorted(set(temps.keys()))
            if self.PublishIndividualTopics:
                msgTStamp = PsutilTemperatureStamped()
            for device in device_names:
                msgT = PsutilTemperatureDevice()
                msgT.device_name = str(device)
                for temp in range(len(temps[device])):
                    temperature_name = str(temps[device][temp].label)
                    if ((temperature_name in self.TemperatureNameMatch) or
                        not len(self.TemperatureNameMatch)):
                        msgTD = PsutilTemperature()
                        msgTD.temperature_name = temperature_name
                        msgTD.current = float(temps[device][temp].current or 0)
                        msgTD.high = float(temps[device][temp].high or 0)
                        msgTD.critical = float(temps[device][temp].critical or 0)
                        msgT.temperatures.append(msgTD)

                if self.PublishIndividualTopics:
                    msgTStamp.temperatures.append(msgT)
                else:
                    msg.temperatures.append(msgT)

            if self.PublishIndividualTopics:
                msgTStamp.header=msgHeader
                self.TemperaturePub.publish(msgTStamp)

        if ((self.PublishProcessor and self.PublishIndividualTopics) or
            not self.PublishIndividualTopics):
            time_since_boot = time.time()-self.boot_time
            msgTime=Time()
            msgTime.sec = int(time_since_boot)
            msgTime.nanosec = int((time_since_boot - msgTime.sec) * 1e9)

        # Processor
        if self.PublishProcessor:
            if self.PublishIndividualTopics:
                msgPStamp = PsutilProcessorStamped()
            if self.PublishProcessorFrequency:
                cpu_freqs = psutil.cpu_freq(percpu=True)
                if not self.PublishProcessorPercent:
                    cpu_iter = len(cpu_freqs)
            if self.PublishProcessorPercent:
                cpu_percent = psutil.cpu_times_percent(percpu=True)
                cpu_iter = len(cpu_percent)
            msgP = PsutilProcessor()
            for i in range(cpu_iter):
                if self.PublishProcessorPercent:
                    percent=cpu_percent[i]
                    msgPP = PsutilProcessorPercent()
                    msgPP.user = float(percent.user or 0)
                    msgPP.nice = float(percent.nice or 0)
                    msgPP.system = float(percent.system or 0)
                    msgPP.idle = float(percent.idle or 0)
                    msgPP.iowait = float(percent.iowait or 0)
                    msgPP.irq = float(percent.irq or 0)
                    msgPP.softirq = float(percent.softirq or 0)
                    msgPP.steal = float(percent.steal or 0)
                    msgPP.guest = float(percent.guest or 0)
                    msgPP.guest_nice = float(percent.guest_nice or 0)
                    msgP.percent = msgPP
                if self.PublishProcessorFrequency:
                    if i < len(cpu_freqs):
                        frequency = cpu_freqs[i]
                    else:
                        frequency = cpu_freqs[-1]
                    msgPF = PsutilProcessorFrequency()
                    msgPF.current = float(frequency.current or 0)
                    msgPF.min = float(frequency.min or 0)
                    msgPF.max = float(frequency.max or 0)
                    msgP.frequency = msgPF

                if self.PublishIndividualTopics:
                    msgPStamp.processors.append(msgP)
                else:
                    msg.processors.append(msgP)

            if self.PublishIndividualTopics:
                msgPStamp.header=msgHeader
                msgPStamp.time_since_boot=msgTime
                msgPStamp.load_average=psutil.getloadavg()
                msgPStamp.cpu_physical_cores = self.physical_cores
                msgPStamp.cpu_logical_cores = self.logical_cores
                self.ProcessorPub.publish(msgPStamp)
            else:
                msg.load_average=psutil.getloadavg()
                msg.cpu_physical_cores = self.physical_cores
                msg.cpu_logical_cores = self.logical_cores

        if not self.PublishIndividualTopics:
            msg.header=msgHeader
            msg.time_since_boot=msgTime
            self.PsutilPub.publish(msg)
        return

def main():
    rclpy.init()
    PSN = PsutilNode()
    rclpy.spin(PSN)
    PSN.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()