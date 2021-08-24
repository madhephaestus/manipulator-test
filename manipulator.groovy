import com.neuronrobotics.bowlerstudio.BowlerStudio
import com.neuronrobotics.bowlerstudio.physics.TransformFactory
import com.neuronrobotics.sdk.addons.kinematics.math.RotationNR
import com.neuronrobotics.sdk.addons.kinematics.math.TransformNR

import eu.mihosoft.vrl.v3d.CSG
import eu.mihosoft.vrl.v3d.Cylinder
import eu.mihosoft.vrl.v3d.Vector3d
import javafx.application.Platform
import javafx.event.EventHandler
import javafx.event.EventType
import javafx.scene.input.MouseButton
import javafx.scene.input.MouseEvent
import javafx.scene.transform.Affine

Affine manipulationMatrix= new Affine();

//Your code here

CSG manip = new Cylinder(20,40).toCSG()


HashMap<EventType<MouseEvent>,EventHandler<MouseEvent>> map=new HashMap<>()
double startx=0;
double starty=0;

double x3d=0;
double y3d=0;
double z3d=0;
double newx=0;
double newy=0;
double newz=0;
Vector3d orintation = new Vector3d(0,1,0);
TransformNR camFrame=null;
boolean dragging=false;
double depth =0;

map.put(MouseEvent.MOUSE_PRESSED,  new EventHandler<MouseEvent>() {
			@Override
			public void handle(MouseEvent event) {
				camFrame= BowlerStudio.getCamerFrame()
				depth=-1200 /BowlerStudio.getCamerDepth()
				event.consume()
				dragging=false;
			}
		})

map.put(MouseEvent.MOUSE_DRAGGED,  new EventHandler<MouseEvent>() {
	@Override
	public void handle(MouseEvent event) {
		if(dragging==false) {
			startx=event.screenX;
			starty=event.screenY;
		}
		dragging=true;
		double deltx=(startx-event.screenX);
		double delty=(starty-event.screenY)
		TransformNR trans=new TransformNR(deltx / depth,
							delty / depth, 0, new RotationNR());
						
		TransformNR global = camFrame.times(trans);
		newx=(global.getX()+x3d)*orintation.getX();
		newy=(global.getY()+y3d)*orintation.getY();
		newz=(global.getZ()+z3d)*orintation.getZ();
		global.setX(newx)
		global.setY(newy)
		global.setZ(newz)
		
		global.setRotation(new RotationNR())
		Platform.runLater({
			TransformFactory.nrToAffine(global, manipulationMatrix)
		})
		double dist = Math.sqrt(Math.pow(deltx, 2)+Math.pow(delty, 2))
		System.out.println(" drag "+global.getX()+" , "+global.getY()+" , "+global.getZ()+" "+deltx+" "+delty);
		
		event.consume()
	}
})

map.put(MouseEvent.MOUSE_RELEASED,  new EventHandler<MouseEvent>() {
	@Override
	public void handle(MouseEvent event) {
		if(dragging) {
			dragging=false;
			z3d=newz
			y3d=newy
			x3d=newx
			event.consume()
		}
	}
})
manip.getStorage().set("manipulator",map)
manip.setManipulator(manipulationMatrix)
return manip