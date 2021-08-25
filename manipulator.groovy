import com.neuronrobotics.bowlerstudio.BowlerStudio
import com.neuronrobotics.bowlerstudio.physics.TransformFactory
import com.neuronrobotics.sdk.addons.kinematics.math.RotationNR
import com.neuronrobotics.sdk.addons.kinematics.math.TransformNR
import com.neuronrobotics.sdk.common.Log

import eu.mihosoft.vrl.v3d.CSG
import eu.mihosoft.vrl.v3d.Cylinder
import eu.mihosoft.vrl.v3d.Vector3d
import javafx.application.Platform
import javafx.event.EventHandler
import javafx.event.EventType
import javafx.scene.input.MouseButton
import javafx.scene.input.MouseEvent
import javafx.scene.paint.Color
import javafx.scene.transform.Affine
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.reflect.TypeToken;
import com.neuronrobotics.bowlerstudio.scripting.ScriptingEngine

import java.lang.reflect.Type;

import org.apache.commons.io.FileUtils
import org.apache.commons.io.IOUtils

TransformNR p = new TransformNR()
Affine m= new Affine();

def event = new Runnable() {
	public void run() {
		Log.enableSystemPrint(true)
		println "New pose found at "+p
	}
}
def eventMoving = new Runnable() {
	public void run() {
		Log.enableSystemPrint(true)
		println "Moving "
	}
}

class manipulation {
	HashMap<EventType<MouseEvent>,EventHandler<MouseEvent>> map=new HashMap<>()
	double startx=0;
	double starty=0;
	double newx=0;
	double newy=0;
	double newz=0;
	TransformNR camFrame=null;
	boolean dragging=false;
	double depth =0;
	public manipulation(Affine manipulationMatrix,Vector3d orintation,CSG manip,TransformNR globalPose,Runnable eve,Runnable moving) {
		map.put(MouseEvent.MOUSE_PRESSED,  new EventHandler<MouseEvent>() {
			@Override
			public void handle(MouseEvent event) {
				new Thread({
					camFrame= BowlerStudio.getCamerFrame()
					depth=-1600 /BowlerStudio.getCamerDepth()
					event.consume()
					dragging=false;
					
				}).start();

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
				newx=(global.getX()*orintation.getX()+globalPose.getX());
				newy=(global.getY()*orintation.getY()+globalPose.getY());
				newz=(global.getZ()*orintation.getZ()+globalPose.getZ());
				global.setX(newx)
				global.setY(newy)
				global.setZ(newz)
				
				global.setRotation(new RotationNR())
				Platform.runLater({
					TransformFactory.nrToAffine(global, manipulationMatrix)
				})
				double dist = Math.sqrt(Math.pow(deltx, 2)+Math.pow(delty, 2))
				//System.out.println(" drag "+global.getX()+" , "+global.getY()+" , "+global.getZ()+" "+deltx+" "+delty);
				moving.run()
				event.consume()
			}
		})
		
		map.put(MouseEvent.MOUSE_RELEASED,  new EventHandler<MouseEvent>() {
			@Override
			public void handle(MouseEvent event) {
				if(dragging) {
					dragging=false;
					globalPose.setX(newx)
					globalPose.setY(newy)
					globalPose.setZ(newz)
					event.consume()
					eve.run()
				}
			}
		})
		manip.getStorage().set("manipulator",map)
		manip.setManipulator(manipulationMatrix)
	}
	
}

class CartesianManipulator{
	CSG manip1 = new Cylinder(0,5,40,10).toCSG()
			.setColor(Color.BLUE)
	CSG manip2 = new Cylinder(0,5,40,10).toCSG()
		.roty(-90)
		.setColor(Color.RED)
	CSG manip3 = new Cylinder(0,5,40,10).toCSG()
		.rotx(90)
		.setColor(Color.GREEN)
	public CartesianManipulator(Affine manipulationMatrix,TransformNR globalPose,Runnable ev,Runnable moving) {
		new manipulation( manipulationMatrix, new Vector3d(0,0,1), manip1, globalPose,ev,moving)
		new manipulation( manipulationMatrix, new Vector3d(0,1,0), manip3, globalPose,ev,moving)
		new manipulation( manipulationMatrix, new Vector3d(1,0,0), manip2, globalPose,ev,moving)
	}
	public ArrayList<CSG> get(){
		return [manip1,manip2,manip3]
	}
}
class BezierEditor{
	Type TT_mapStringString = new TypeToken<HashMap<String, List<Double>>>() {}.getType();
	Gson gson = new GsonBuilder().disableHtmlEscaping().setPrettyPrinting().create();
	File	cachejson;
	public BezierEditor(File data) {
		cachejson = data
		String jsonString = null;
		InputStream inPut = null;
		inPut = FileUtils.openInputStream(cachejson);
		jsonString = IOUtils.toString(inPut);
		HashMap<String, List<Double>> database = gson.fromJson(jsonString, TT_mapStringString);
		for(String key:database.keySet()) {
			List<Double> value = database.get(key)
			println key+" "+value
		}
	}
}

File	cachejson = ScriptingEngine.fileFromGit("https://github.com/madhephaestus/manipulator-test.git", "bez.json")
BezierEditor editor = new BezierEditor(cachejson)


def cart=new CartesianManipulator(m,p,event,eventMoving)

return cart.get()