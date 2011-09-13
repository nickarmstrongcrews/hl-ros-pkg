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

import java.net.URI;
import java.util.List;

import android.app.Activity;
import android.content.Context;
import android.hardware.SensorManager;
import android.location.Location;
import android.location.LocationManager;
import android.os.Bundle;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeRunner;
import org.ros.rosjava.android.MessageCallable;
import org.ros.rosjava.android.views.RosTextView;

/**
 * @author nick@heuristiclabs.com (Nick Armstrong-Crews)
 */
public class MainActivity extends Activity {

  private final NodeRunner nodeRunner;
  
  private RosTextView<org.ros.message.std_msgs.String> rosTextView;
  private GlobalPosePublisher globalPosePub;
   
  protected LocationManager locMan;
  
  public MainActivity() {
	super();
    nodeRunner = NodeRunner.newDefault();
  }
  
  @SuppressWarnings("unchecked")
  @Override
  public void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.main);
    locMan = (LocationManager) getSystemService(Context.LOCATION_SERVICE);
    rosTextView = (RosTextView<org.ros.message.std_msgs.String>) findViewById(R.id.text);
    rosTextView.setTopicName("/chatter");
    rosTextView.setMessageType("std_msgs/String");
    rosTextView
        .setMessageToStringCallable(new MessageCallable<String, org.ros.message.std_msgs.String>() {
          @Override
          public String call(org.ros.message.std_msgs.String message) {
        	String s = "";
        	List<String> providers = locMan.getAllProviders();
        	for(String provider : providers) {
        		s += provider;
        		if(locMan.isProviderEnabled(provider)) s += ":enabled\n";
        		else s += ":disabled\n";
        	}
        	//String chosenProvider = "gps";
        	String chosenProvider = "network";
        	if(locMan.isProviderEnabled(chosenProvider)) {
            	Location loc = locMan.getLastKnownLocation(chosenProvider);
            	//s = loc.toString();
            	double lati = loc.getLatitude();
            	double longi = loc.getLongitude();
            	double accuracy = loc.getAccuracy();
            	s = "lat: " + lati + ", long: " + longi + ", accuracy: " + accuracy;
        	}
        	  return s;
          }
        });
  }
  
  @Override
  protected void onPause() {
    super.onPause();
    rosTextView.shutdown();
    globalPosePub.shutdown();
  }
    
  @Override
  protected void onResume() {
    super.onResume();
    try {
      URI masterUri = new URI("http://192.168.43.171:11311");
      NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic("192.168.43.1", masterUri);
      globalPosePub =
          new GlobalPosePublisher(
      		  	(SensorManager) getSystemService(SENSOR_SERVICE),
      		  	(LocationManager) getSystemService(LOCATION_SERVICE)
      		  );
      nodeRunner.run(globalPosePub, nodeConfiguration);
      nodeRunner.run(rosTextView, nodeConfiguration);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
  }

}
