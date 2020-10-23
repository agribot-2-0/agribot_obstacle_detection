#!/usr/bin/env python
import can
import rospy

from sensor_msgs.msg import Range


class SocketcanSonarReader:
    def __init__(self, can_interface='can0'):
        self.bus = can.ThreadSafeBus(can_interface, bustype='socketcan')

        # Motec CAN ids
        self.motec_config_id = 0x220
        self.motec_request_id = 0x240
        self.motec_response_id = 0x340

        # Motec CAN sub-IDs
        self.motec_can_output_sub_id = 0xAF

        # Balluf CAN id
        self.balluff_id = 0x1000

        # min/max range and field of view
        self.balluf_min_range = 0.35
        self.balluf_max_range = 5.0
        self.balluf_fov = 0.27576
        self.motec_min_range = 0.15
        self.motec_max_range = 3.0
        self.motec_fov = 0.523598

        # ROS publishers
        self.balluff_publisher = rospy.Publisher('/bumper/front/sonar/center',
                                                 Range,
                                                 queue_size=1)
        self.balluff_frame_id = 'bumper_front_sonar_center_link'

        self.motec_publishers = {
            5: rospy.Publisher('/bumper/front/sonar/left_side/rear', Range,
                               queue_size=1),
            6: rospy.Publisher('/bumper/front/sonar/left_side/front', Range,
                               queue_size=1),
            7: rospy.Publisher('/bumper/front/sonar/left', Range,
                               queue_size=1),
            8: rospy.Publisher('/bumper/front/sonar/right', Range,
                               queue_size=1),
            9: rospy.Publisher('/bumper/front/sonar/right_side/front', Range,
                               queue_size=1),
            10: rospy.Publisher('/bumper/front/sonar/right_side/rear', Range,
                               queue_size=1),
        }

        self.motec_frame_ids = {
            5: 'bumper_front_sonar_left_side_rear_link',
            6: 'bumper_front_sonar_left_side_front_link',
            7: 'bumper_front_sonar_left_link',
            8: 'bumper_front_sonar_right_link',
            9: 'bumper_front_sonar_right_side_front_link',
            10: 'bumper_front_sonar_right_side_rear_link',
        }

    def parse_balluff_sonar_msg(self, msg):
        range = self.balluff_range_from_bytes(int(msg.data[0]),
                                              int(msg.data[1]))
        self.publish_balluff_range(0x00, range)

    def parse_motec_sonar_msg(self, msg):
        sub_id = msg.data[0]

        if sub_id == self.motec_can_output_sub_id:
            self.parse_motec_can_output_msg(msg)
        else:
            rospy.logerr(
                'Received Motec CAN msg with unsupported sub-ID [{}]'.format(
                    msg.arbitration_id))

    def parse_motec_can_output_msg(self, msg):
        # Check data length
        if len(msg.data) != 4:
            rospy.loginfo(
                'Motec measurement data length is {}. Should be 4. Ignoring msg...'
                .format(len(msg.data)))
            return

        sensor_id = int(msg.data[1])
        range = self.motec_range_from_bytes(int(msg.data[2]), int(msg.data[3]), sensor_id)
        self.publish_motec_range(sensor_id, range)

    def parse_msg(self, msg):
        if msg.arbitration_id == self.balluff_id:
            self.parse_balluff_sonar_msg(msg)
        elif msg.arbitration_id == self.motec_response_id:
            self.parse_motec_sonar_msg(msg)
        else:
            rospy.logerr('Received CAN msg with unsupported ID [{}]'.format(
                msg.arbitration_id))

    def motec_range_from_bytes(self, lsb, msb, sensor_id):
        if lsb == 0xFE and msb == 0xFF:
            # No object detected
            return float('inf')
        elif lsb == 0xFF and msb == 0xFF:
            # ERROR - no sensor detected
            rospy.logerr('Motec sensor #{} not detected...'.format(sensor_id))
            return -self.motec_max_range
        else:
            return (lsb | msb << 8) / 100.0

    def balluff_range_from_bytes(self, lsb, msb):
        adc_value = lsb | msb << 8
        range = self.balluf_min_range + (adc_value / 700.0)
        return range

    def publish_motec_range(self, id, range):
        if id in self.motec_frame_ids and id in self.motec_publishers:
            msg = Range()
            msg.header.frame_id = self.motec_frame_ids[id]
            msg.header.stamp = rospy.Time.now()

            msg.radiation_type = Range.ULTRASOUND
            msg.min_range = self.motec_min_range
            msg.max_range = self.motec_max_range
            msg.field_of_view = self.motec_fov
            msg.range = range

            pub = self.motec_publishers[id]
            pub.publish(msg)
        else:
            rospy.logwarn('No motec defined for id={}'.format(id))

    def publish_balluff_range(self, id, range):
        msg = Range()
        msg.header.frame_id = self.balluff_frame_id
        msg.header.stamp = rospy.Time.now()

        msg.radiation_type = Range.ULTRASOUND
        msg.min_range = self.balluf_min_range
        msg.max_range = self.balluf_max_range
        msg.field_of_view = self.balluf_fov
        msg.range = range

        pub = self.balluff_publisher
        pub.publish(msg)

    def run(self):
        while not rospy.is_shutdown():
            msg = self.bus.recv()
            self.parse_msg(msg)


if __name__ == '__main__':
    rospy.init_node('socketcan_sonar_reader_node')
    ssr = SocketcanSonarReader()
    ssr.run()
