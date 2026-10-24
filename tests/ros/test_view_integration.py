"""Real ROS PointCloud2/OccupancyGrid/TF to shared Qt views, without hardware."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import pytest
pytestmark=pytest.mark.skipif(os.environ.get('AGT_RUN_ROS_INTEGRATION')!='1',reason='requires ROS Humble and Qt')

PUBLISHER='''
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy,qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2,PointField
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import struct
rclpy.init();node=Node('workbench_view_fixture')
cloud=node.create_publisher(PointCloud2,'/test/live_cloud',qos_profile_sensor_data)
grid=node.create_publisher(OccupancyGrid,'/test/map',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))
tf=TransformBroadcaster(node)
def publish():
    stamp=node.get_clock().now().to_msg()
    for child,x in [('base_link',2.),('fixture_sensor',1.)]:
        value=TransformStamped();value.header.stamp=stamp;value.header.frame_id='map';value.child_frame_id=child
        value.transform.translation.x=x;value.transform.rotation.w=1.;tf.sendTransform(value)
    m=PointCloud2();m.header.stamp=stamp;m.header.frame_id='fixture_sensor';m.height=1;m.width=6500
    m.fields=[PointField(name=axis,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,axis in enumerate(('x','y','z'))]
    m.point_step=12;m.row_step=m.width*12;m.data=b''.join(struct.pack('<fff',i*.001,0.,i*.0001) for i in range(6500));cloud.publish(m)
    g=OccupancyGrid();g.header.stamp=stamp;g.header.frame_id='map';g.info.width=20;g.info.height=20;g.info.resolution=.1
    g.info.origin.orientation.w=1.;g.data=[0]*400;grid.publish(g)
node.create_timer(.05,publish)
rclpy.spin(node)
'''


def test_real_tf_grid_cloud_update_all_views(tmp_path):
    from PyQt5.QtWidgets import QApplication
    from PyQt5.QtTest import QTest
    from agt_robot_studio.ui import GridView,DEFAULT_CONFIG
    from agt_robot_studio.tri_views import TriViewWorkbench
    from agt_robot_studio.ros_worker import RosWorker
    app=QApplication.instance() or QApplication([])
    script=tmp_path/'publisher.py';script.write_text(PUBLISHER)
    with (tmp_path/'publisher.log').open('w') as log:
        publisher=subprocess.Popen([sys.executable,str(script)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    worker=RosWorker(dict(DEFAULT_CONFIG,live_points_topic='/test/live_cloud',map_topic='/test/map'))
    notices=[];worker.notice.connect(notices.append)
    view=TriViewWorkbench(GridView(True));view.resize(900,550);view.show()
    worker.state.connect(lambda s:view.set_robot(s.get('map_robot')))
    worker.start();deadline=time.monotonic()+10;updates=0
    try:
        while time.monotonic()<deadline:
            QTest.qWait(50)
            for kind,frame in worker.frames.take().items():
                if kind=='cloud':view.set_cloud(frame);updates+=1
                if kind=='grid':view.grid.set_grid(frame)
            if updates>=2 and view.top.pose and not view.grid.image.isNull():break
        assert updates>=2 and view.top.snapshot is view.side.snapshot,notices
        assert view.top.snapshot['frame_id']=='map'
        assert 1.<=view.top.snapshot['xyz'][0,0]<1.01  # real timestamped TF applied once
        assert view.top.pose['x']==2. and not view.grid.image.isNull()
        assert view.top.pose['frame_id']==view.grid.frame_id=='map'
        assert not view.grab().isNull()
    finally:
        worker.shutdown();assert worker.wait(4000)
        if publisher.poll() is None:os.killpg(publisher.pid,signal.SIGINT);publisher.wait(timeout=4)
        view.close()
