 package se.oru.aass.lucia_meta_csp_lecture.executionMonitoring;

import org.apache.commons.logging.Log;
import org.metacsp.dispatching.DispatchingFunction;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;
import org.ros.message.MessageListener;

public class ROSTopicListener extends AbstractNodeMain {

	private String robotName;
	private ROSDispatchingFunction df;
	
	public ROSTopicListener(String robotName, ROSDispatchingFunction df) {
		this.robotName = robotName;
		this.df = df;
	}
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("LuciaListener");
	}

	@Override
	public void onStart(ConnectedNode connectedNode) {
		System.out.println("Hello there, I am running!!");
		final Log log = connectedNode.getLog();
		Subscriber<std_msgs.String> subscriber = connectedNode.newSubscriber("/" + robotName + "/move_command", std_msgs.String._TYPE);
		subscriber.addMessageListener(new MessageListener<std_msgs.String>() {
			@Override
			public void onNewMessage(std_msgs.String message) {
				if (message.getData().equals("SUCCESS") || message.getData().equals("FAILURE")) {
					df.finishCurrentActivity();
					df.setExecuting(false);
				}
			}
		});
	}


}
