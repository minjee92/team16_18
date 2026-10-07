import sys,types,threading,time,unittest
from unittest.mock import Mock,patch
import numpy as np
from pinky_lane import config
from pinky_lane.config import *
from pinky_lane.core.types import *
from pinky_lane.control.command_gate import CommandGate
from pinky_lane.perception.lane_estimator import LaneEstimator
from pinky_lane.perception.intersection_detector import IntersectionDetector
c = types.SimpleNamespace(**{k:v for k,v in globals().copy().items() if not k.startswith('_')})

# Import the ROS adapter without loading ROS, camera or inference packages.
class Twist:
 def __init__(self):self.linear=types.SimpleNamespace(x=0.);self.angular=types.SimpleNamespace(z=0.)
for name in ['rclpy','rclpy.node','rclpy.clock','rclpy.callback_groups','rclpy.executors','geometry_msgs','geometry_msgs.msg','nav_msgs','nav_msgs.msg','std_msgs','std_msgs.msg','tf2_ros']:
 sys.modules[name]=types.ModuleType(name)
sys.modules['rclpy.node'].Node=object
sys.modules['rclpy.executors'].MultiThreadedExecutor=object
sys.modules['rclpy.clock'].Clock=object
sys.modules['rclpy.clock'].ClockType=types.SimpleNamespace(STEADY_TIME=1)
sys.modules['rclpy.callback_groups'].MutuallyExclusiveCallbackGroup=object
sys.modules['geometry_msgs.msg'].PoseStamped=object
sys.modules['geometry_msgs.msg'].Twist=Twist
sys.modules['nav_msgs.msg'].Odometry=object
sys.modules['std_msgs.msg'].Bool=object
sys.modules['tf2_ros'].Buffer=object;sys.modules['tf2_ros'].TransformListener=object
import pinky_lane.ros.node as app

def bare_node():
 n=app.LaneMissionController.__new__(app.LaneMissionController)
 n._data_lock=threading.Lock();n._gate=c.CommandGate(.7);n._enabled=False;n._enabled_since=None
 n._closed=False;n._arming_epoch=0;n._seen_arming_epoch=0;n._seen_epoch=0;n._pending_goal=False
 n._intersection=Mock();n._enter_start_pose=None;n._turn_start_yaw=None
 n._cmd_pub=Mock();n._odom_stamp=time.monotonic();n._goal_xy=None;n._goal_yaw=None
 n._state=c.DriveState.FOLLOW;n._state_since=time.monotonic();n._note='';n._lane=Mock()
 n.get_logger=Mock(return_value=Mock());n._snapshot=Mock(return_value=(None,None,True,1.,(0.,0.,0.),time.monotonic()))
 n._goal_distance=Mock(return_value=None)
 return n

class CoreTests(unittest.TestCase):
 def test_one_row_does_not_reset_loss(self):
  lane=c.LaneEstimator(640,480);mask=np.zeros((480,640),np.uint8);mask[430:435,100:105]=1
  for t in [1.,1.2,2.,3.1]:result=lane.update(mask,None,t)
  self.assertFalse(result.valid);self.assertEqual(result.lost_frames,4);self.assertGreaterEqual(result.lost_seconds,2.)
  n=bare_node();self.assertEqual(n._control(result,False,time.monotonic()),(0.,0.))
 def test_startup_without_lanes_does_not_move(self):
  lane=c.LaneEstimator(640,480);result=lane.update(None,None,1.)
  self.assertFalse(result.can_coast);self.assertEqual(bare_node()._control(result,False,time.monotonic()),(0.,0.))
 def test_loss_after_valid_lane_coasts_only_temporarily(self):
  lane=c.LaneEstimator(640,480);mask=np.zeros((480,640),np.uint8);mask[:,100]=1
  lane.update(mask,None,1.);self.assertTrue(lane.update(None,None,1.1).can_coast)
  self.assertFalse(lane.update(None,None,3.2).can_coast)
 def test_reversed_labels_are_not_a_valid_pair(self):
  lane=c.LaneEstimator(640,480);left=np.zeros((480,640),np.uint8);right=left.copy();left[:,500]=1;right[:,100]=1
  self.assertFalse(lane.update(left,right,1.).valid)
 def test_valid_detection_recovers_and_single_lane_works(self):
  lane=c.LaneEstimator(640,480);lane.update(None,None,1.)
  mask=np.zeros((480,640),np.uint8);mask[:,100]=1
  result=lane.update(mask,None,2.);self.assertTrue(result.valid);self.assertEqual(result.lost_frames,0)
 def test_debounce_requires_consecutive_frames(self):
  d=c.IntersectionDetector()
  self.assertFalse(d.update(.03,1));self.assertFalse(d.update(.03,2));self.assertFalse(d.update(0,3));self.assertFalse(d.update(.03,4));self.assertFalse(d.update(.03,5));self.assertTrue(d.update(.03,6))
  self.assertFalse(d.update(.03,20));d.update(0,21)
  self.assertFalse(d.update(.03,22));self.assertFalse(d.update(.03,23));self.assertTrue(d.update(.03,24))
 def test_gate_rejects_pre_disable_work(self):
  g=c.CommandGate(.7);g.set_enabled(True);epoch=g.epoch;g.submit(epoch,10,.1,.2)
  self.assertEqual(g.current(10.5),(.1,.2));g.set_enabled(False);g.set_enabled(True)
  self.assertFalse(g.submit(epoch,10.6,.1,.2));self.assertEqual(g.current(10.6),(0.,0.))
 def test_gate_uses_capture_age_and_rejects_nonfinite(self):
  g=c.CommandGate(.7);g.set_enabled(True);g.submit(g.epoch,10,.1,0.)
  self.assertEqual(g.current(10.8),(0.,0.));self.assertEqual(g.current(9.),(0.,0.))
  self.assertFalse(g.submit(g.epoch,11,float('nan'),0.))
 def test_entry_timeout_allows_nominal_distance(self):
  self.assertGreater(c.MISSION.enter_timeout_s,c.MISSION.enter_distance_m/c.MOTION.min_speed)

class AdapterTests(unittest.TestCase):
 def test_disable_and_timer_publish_zero(self):
  n=bare_node()
  with patch.object(config,'DRIVE_OUTPUT',True):
   n._on_enable(types.SimpleNamespace(data=True));epoch=n._gate.epoch
   n._send_velocity(.1,.3,epoch,time.monotonic());n._output_tick()
   self.assertEqual(n._cmd_pub.publish.call_args.args[0].linear.x,.1)
   n._on_enable(types.SimpleNamespace(data=False));n._send_velocity(.1,.3,epoch,time.monotonic());n._output_tick()
   self.assertEqual(n._cmd_pub.publish.call_args.args[0].linear.x,0.)
 def test_dry_run_never_publishes(self):
  n=bare_node()
  with patch.object(config,'DRIVE_OUTPUT',False):n._output_tick();n._publish_zero();n._on_enable(types.SimpleNamespace(data=False))
  n._cmd_pub.publish.assert_not_called()
 def test_repeated_enable_does_not_reset_arming(self):
  n=bare_node();n._on_enable(types.SimpleNamespace(data=True));stamp=n._enabled_since;epoch=n._gate.epoch
  n._on_enable(types.SimpleNamespace(data=True));self.assertEqual(n._enabled_since,stamp);self.assertEqual(n._gate.epoch,epoch)
 def test_disable_reenable_between_frames_rearms_fault(self):
  n=bare_node();n._state=c.DriveState.FAULT
  n._on_enable(types.SimpleNamespace(data=True));n._consume_requests()
  self.assertEqual(n._state,c.DriveState.FOLLOW)
  n._state=c.DriveState.FAULT
  n._on_enable(types.SimpleNamespace(data=False));n._on_enable(types.SimpleNamespace(data=True));n._consume_requests()
  self.assertEqual(n._state,c.DriveState.FOLLOW)
 def test_goal_change_during_turn_holds_fault(self):
  n=bare_node();n._on_enable(types.SimpleNamespace(data=True));n._consume_requests();n._state=c.DriveState.TURNING
  goal=types.SimpleNamespace(header=types.SimpleNamespace(frame_id='map'),pose=types.SimpleNamespace(position=types.SimpleNamespace(x=1.,y=2.)))
  n._on_goal(goal);n._consume_requests();self.assertEqual(n._state,c.DriveState.FAULT)
 def test_bad_goal_frame_is_rejected(self):
  n=bare_node();n._on_goal(types.SimpleNamespace(header=types.SimpleNamespace(frame_id='odom')));self.assertIsNone(n._goal_xy)
 def test_turn_timeout_stays_fault(self):
  n=bare_node();n._state=c.DriveState.TURNING;n._state_since=0;n._turn_start_yaw=0.;n._turn_target=-1.57
  self.assertEqual(n._run_turn(10.,(0,0,0),9.9),(0.,0.));self.assertEqual(n._state,c.DriveState.FAULT)
  lane=c.LaneEstimate([],0.,1.,True,0,320.)
  self.assertEqual(n._control(lane,False,time.monotonic()),(0.,0.));self.assertEqual(n._state,c.DriveState.FAULT)
 def test_mask_original_coordinate_polygons(self):
  cls=Mock();cls.cpu.return_value.numpy.return_value=np.array([0])
  r=types.SimpleNamespace(names={0:'left_lane'},boxes=types.SimpleNamespace(cls=cls),masks=types.SimpleNamespace(xy=[np.array([[10,10],[20,10],[20,20],[10,20]])]))
  mask=app.segmentation_masks(r,640,480)['left_lane'][0];self.assertEqual(mask.shape,(480,640));self.assertEqual(mask[15,15],1);self.assertEqual(mask[30,30],0)

if __name__=='__main__':unittest.main()
