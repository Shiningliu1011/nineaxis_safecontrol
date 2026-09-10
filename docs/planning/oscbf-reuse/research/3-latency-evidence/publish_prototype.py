"""Isolated DDS serialization/publish-call microprobe; never a robot command topic."""
import json,time,os
from pathlib import Path
import numpy as np
import rclpy
from rclpy.context import Context
from rclpy.serialization import serialize_message
from sensor_msgs.msg import JointState
ctx=Context(); rclpy.init(context=ctx,domain_id=181)
node=rclpy.create_node('wayfinder_latency_probe',context=ctx)
pub=node.create_publisher(JointState,'/wayfinder_profile3/offline_probe',10)
message=JointState(); message.name=[f'J{i}' for i in range(1,10)]; message.position=[0.]*9
out={'domain_id':181,'topic':'/wayfinder_profile3/offline_probe','scope':'publisher call return only; no subscriber, DDS delivery/CAN/actuator not measured'}
for name,fn in [('serialization',lambda:serialize_message(message)),('publish_call',lambda:pub.publish(message))]:
    for _ in range(20):fn()
    samples=[]
    for _ in range(300):
        start=time.perf_counter_ns();fn();samples.append((time.perf_counter_ns()-start)/1e6)
    out[name]={'samples_ms':samples,'p50_ms':float(np.percentile(samples,50)),'p95_ms':float(np.percentile(samples,95)),'p99_ms':float(np.percentile(samples,99)),'max_ms':max(samples)}
node.destroy_node();rclpy.shutdown(context=ctx)
Path(__file__).with_name('publish.json').write_text(json.dumps(out,indent=2))
