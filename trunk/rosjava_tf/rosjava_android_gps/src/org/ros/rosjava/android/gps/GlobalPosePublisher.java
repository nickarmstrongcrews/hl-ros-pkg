/*
 * Copyright 2011, Heuristic Labs, LLC
 * 
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package org.ros.rosjava.android.gps;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.location.Criteria;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.os.Bundle;

import org.ros.message.Time;
import org.ros.message.geometry_msgs.PoseStamped;
import org.ros.message.geometry_msgs.Quaternion;
import org.ros.node.DefaultNodeFactory;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;

/**
 * @author nick@heuristiclabs.com (Nick Armstrong-Crews)
 */
public class GlobalPosePublisher implements NodeMain {

  protected final LocationManager locMan;
	  
  private final SensorManager sensorManager;

  private Node node;
  private OrientationListener orientationListener;

  private static String [] providerPriorityList = {"gps","network"};
  
  private final class OrientationListener implements SensorEventListener {

	protected String provider;

	private final Publisher<PoseStamped> publisher;
	private org.ros.message.geometry_msgs.Point origin;
	private volatile int seq;
	  
    private OrientationListener(Publisher<PoseStamped> publisher) {

    	this.publisher = publisher;
    	origin = null;
    	seq = 0;

    	//provider = "network";
    	for(String p : providerPriorityList) {
    		if(locMan.isProviderEnabled(p)) {
    			Location loc = locMan.getLastKnownLocation(p);
    			if(loc != null) {
    				provider = p;
    				break;
    			}
    		}
    	}

    }

    @Override
    public void onSensorChanged(SensorEvent event) {
      if (event.sensor.getType() == Sensor.TYPE_ROTATION_VECTOR) {

    	  if(provider != null) {
	      	Location loc = locMan.getLastKnownLocation(provider);
	      	if(origin == null) origin = new org.ros.message.geometry_msgs.Point();
	      	origin.x = loc.getLatitude();
	    	origin.y = loc.getLongitude();
	    	origin.z = 0;
    	}
 
    	if(origin != null) {
    	
	        float[] quaternion = new float[4];
	        SensorManager.getQuaternionFromVector(quaternion, event.values);
	        Quaternion orientation = new Quaternion();
	        orientation.w = quaternion[0];
	        orientation.x = quaternion[1];
	        orientation.y = quaternion[2];
	        orientation.z = quaternion[3];
	        PoseStamped pose = new PoseStamped();
	        pose.header.frame_id = "/earth"; // /map
	        pose.header.seq = seq++;
	        pose.header.stamp = Time.fromMillis(System.currentTimeMillis());
	        pose.pose.position = origin;
	        pose.pose.orientation = orientation;
	        publisher.publish(pose);
    	}

      }

    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {}

  }
  
  public GlobalPosePublisher(SensorManager sensorManager, LocationManager locMan) {
    this.sensorManager = sensorManager;
    this.locMan = locMan;

  }

  @Override
  public void main(NodeConfiguration configuration) throws Exception {
    try {
      node = new DefaultNodeFactory().newNode("android/global_pose_publisher", configuration);
      Publisher<org.ros.message.geometry_msgs.PoseStamped> publisher =
          node.newPublisher("android/global_pose", "geometry_msgs/PoseStamped");
      orientationListener = new OrientationListener(publisher);
      Sensor sensor = sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);
      sensorManager.registerListener(orientationListener, sensor, 500000); // 10
                                                                           // Hz
    } catch (Exception e) {
      if (node != null) {
        node.getLog().fatal(e);
      } else {
        e.printStackTrace();
      }
    }
  }

  @Override
  public void shutdown() {
    sensorManager.unregisterListener(orientationListener);
    node.shutdown();
  }

}
