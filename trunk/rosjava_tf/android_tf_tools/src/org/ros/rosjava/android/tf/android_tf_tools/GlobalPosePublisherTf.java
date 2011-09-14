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

package org.ros.rosjava.android.tf.android_tf_tools;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.location.Location;
import android.location.LocationManager;

import org.ros.node.DefaultNodeFactory;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.rosjava.tf.pubsub.TransformBroadcaster;

/**
 * @author nick@heuristiclabs.com (Nick Armstrong-Crews)
 */
public class GlobalPosePublisherTf implements NodeMain {

  protected final LocationManager locMan;
	  
  private final SensorManager sensorManager;

  private Node node;
  private OrientationListener orientationListener;

  private static String [] providerPriorityList = {"gps","network"};
  
  private final class OrientationListener implements SensorEventListener {

	protected String provider;
	protected org.ros.message.geometry_msgs.Point origin;

	protected OrientationListener(String provider) {
		this.provider = provider;
    	origin = null;
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
      if (event.sensor.getType() == Sensor.TYPE_ROTATION_VECTOR) {

    	  // poll global location
    	  if(provider != null) {
	      	Location loc = locMan.getLastKnownLocation(provider);
	      	if(origin == null) origin = new org.ros.message.geometry_msgs.Point();
	      	origin.x = loc.getLatitude();
	    	origin.y = loc.getLongitude();
	    	if(loc.hasAltitude()) origin.z = loc.getAltitude();
	    	else origin.z = 0;
    	}
 
    	// package transform and send
    	if(origin != null) {
        	float[] q = new float[4];
            SensorManager.getQuaternionFromVector(q, event.values);
            tfb.sendTransform(	"/earth", "/phone",
					((long) System.currentTimeMillis())*1000000, // nanoseconds
					origin.x, origin.y, origin.z,
					q[1], q[2], q[3], q[0] // different order
				);
    	}

      }

    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {}

  }

  protected TransformBroadcaster tfb;

  public GlobalPosePublisherTf(SensorManager sensorManager, LocationManager locMan) {
    this.sensorManager = sensorManager;
    this.locMan = locMan;
  }

  @Override
  public void main(NodeConfiguration configuration) throws Exception {
    try {
    	node = new DefaultNodeFactory().newNode("android/global_pose_publisher", configuration);
		tfb = new TransformBroadcaster(node);

    	String provider = null;
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
		orientationListener = new OrientationListener(provider);

		Sensor sensor = sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);
		sensorManager.registerListener(orientationListener, sensor, 500000); // 10 Hz
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
