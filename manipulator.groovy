import com.neuronrobotics.bowlerstudio.BowlerStudio
import com.neuronrobotics.bowlerstudio.physics.TransformFactory
import com.neuronrobotics.sdk.addons.kinematics.math.RotationNR
import com.neuronrobotics.sdk.addons.kinematics.math.TransformNR
import com.neuronrobotics.sdk.common.Log

import eu.mihosoft.vrl.v3d.CSG
import eu.mihosoft.vrl.v3d.Cylinder
import eu.mihosoft.vrl.v3d.Extrude
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

import eu.mihosoft.vrl.v3d.Transform


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
		Platform.runLater({
			TransformFactory.nrToAffine(globalPose, manipulationMatrix)
		})
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
	public Affine manipulationMatrix= new Affine();
	CSG manip1 = new Cylinder(0,5,40,10).toCSG()
			.setColor(Color.BLUE)
	CSG manip2 = new Cylinder(0,5,40,10).toCSG()
		.roty(-90)
		.setColor(Color.RED)
	CSG manip3 = new Cylinder(0,5,40,10).toCSG()
		.rotx(90)
		.setColor(Color.GREEN)
	public CartesianManipulator(TransformNR globalPose,Runnable ev,Runnable moving) {
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
	TransformNR end = new TransformNR()
	TransformNR cp1 = new TransformNR()
	TransformNR cp2 = new TransformNR()
	ArrayList<CSG> parts = new ArrayList<CSG>()
	CSG displayPart=new Cylinder(5,0,20,10).toCSG()
							.toZMax()
							.roty(-90)
							
	CartesianManipulator endManip;
	CartesianManipulator cp1Manip;
	CartesianManipulator cp2Manip;
	HashMap<String, List<Double>> database;
	boolean updating = false;
	private String url;
	private String gitfile;
	public BezierEditor(String URL, String file, int numPoints) {
		this(ScriptingEngine.fileFromGit(URL, file),numPoints)
		url=URL
		gitfile=file
	}
	public BezierEditor(File data, int numPoints) {
		cachejson = data
		String jsonString = null;
		if(cachejson.exists()) {
			InputStream inPut = null;
			inPut = FileUtils.openInputStream(cachejson);
			jsonString = IOUtils.toString(inPut);
			database = gson.fromJson(jsonString, TT_mapStringString);
			
			List<Double> cp1in = database.get("control one")
			List<Double> cp2in = database.get("control two")
			List<Double> ep = database.get("end point")
			end.setX(ep.get(0))
			end.setY(ep.get(1))
			end.setZ(ep.get(2))
			cp1.setX(cp1in.get(0))
			cp1.setY(cp1in.get(1))
			cp1.setZ(cp1in.get(2))
			cp2.setX(cp2in.get(0))
			cp2.setY(cp2in.get(1))
			cp2.setZ(cp2in.get(2))
		}else {
			end.setX(100)
			end.setY(100)
			end.setZ(100)
			cp1.setX(50)
			cp1.setY(-50)
			cp1.setZ(50)
			cp2.setX(0)
			cp2.setY(50)
			cp2.setZ(-50)
			database= new HashMap<String, List<Double>>()
			
		}
		
		
		endManip=new CartesianManipulator(end,{save()},{update()})
		cp1Manip=new CartesianManipulator(cp1,{save()},{update()})
		cp2Manip=new CartesianManipulator(cp2,{save()},{update()})
		
		for(int i=0;i<numPoints;i++){
			def part=displayPart.clone()
			part.setManipulator(new Affine())
			parts.add(part)
		}
		update()
	}
	public ArrayList<CSG> get(){
		
		ArrayList<CSG> back= new ArrayList<CSG>()
		back.addAll(endManip.get())
		back.addAll(cp1Manip.get())
		back.addAll(cp2Manip.get())
		back.addAll(parts)
		return back
	}
	
	public void update() {
		if(updating) {
			return
		}
		updating=true;
		ArrayList<Transform> transforms = transforms ()
		for(int i=0;i<parts.size();i++) {
			TransformNR nr=TransformFactory.csgToNR(transforms.get(i))
			def partsGetGetManipulator = parts.get(i).getManipulator()
			Platform.runLater({
				TransformFactory.nrToAffine(nr, partsGetGetManipulator)
			})

		}
		updating=false;
	}
	public ArrayList<Transform> transforms (){
		return Extrude.bezierToTransforms(
			new Vector3d(cp1Manip.manipulationMatrix.getTx(),cp1Manip.manipulationMatrix.getTy(),cp1Manip.manipulationMatrix.getTz()), // Control point one
			new Vector3d(cp2Manip.manipulationMatrix.getTx(),cp2Manip.manipulationMatrix.getTy(),cp2Manip.manipulationMatrix.getTz()), // Control point two
			new Vector3d(endManip.manipulationMatrix.getTx(),endManip.manipulationMatrix.getTy(),endManip.manipulationMatrix.getTz()), // Endpoint
			parts.size()// Iterations
			)
	}
	public void save() {
		database.clear()
		database.put("control one",[cp1Manip.manipulationMatrix.getTx(),cp1Manip.manipulationMatrix.getTy(),cp1Manip.manipulationMatrix.getTz()])
		database.put("control two",[cp2Manip.manipulationMatrix.getTx(),cp2Manip.manipulationMatrix.getTy(),cp2Manip.manipulationMatrix.getTz()])
		database.put("end point",[endManip.manipulationMatrix.getTx(),endManip.manipulationMatrix.getTy(),endManip.manipulationMatrix.getTz()])
		database.put("number of points",[parts.size()])
		ArrayList<Transform> transforms = transforms ()
		for(int i=0;i<parts.size();i++) {
			TransformNR nr=TransformFactory.csgToNR(transforms.get(i))
			def partsGetGetManipulator = parts.get(i).getManipulator()
			Platform.runLater({
				TransformFactory.nrToAffine(nr, partsGetGetManipulator)
			})
			String key="pose "+i
			database.put(key,[nr.getX(),nr.getY(),nr.getZ(),
				Math.toDegrees(nr.getRotation().getRotationAzimuth()),
				Math.toDegrees(nr.getRotation().getRotationElevation()),
				Math.toDegrees(nr.getRotation().getRotationTilt())])
		}
		println "Saving to file "+cachejson.getName()
		String writeOut = gson.toJson(database, TT_mapStringString);
		if(url!=null) {
			ScriptingEngine.pushCodeToGit(url, ScriptingEngine.getFullBranch(url), gitfile, writeOut, "Saving Bezier")
		}else {
			if(!cachejson.exists())
				cachejson.createNewFile()
			OutputStream out = null;
			try {
				out = FileUtils.openOutputStream(cachejson, false);
				IOUtils.write(writeOut, out);
				out.close(); // don't swallow close Exception if copy
				// completes
				// normally
			} finally {
				IOUtils.closeQuietly(out);
			}
		}
	}
}

File	cachejson = ScriptingEngine.fileFromGit("https://github.com/madhephaestus/manipulator-test.git", "bez.json")
BezierEditor editor = new BezierEditor("https://github.com/madhephaestus/manipulator-test.git", "bez.json",10)




return editor.get()