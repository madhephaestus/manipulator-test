import eu.mihosoft.vrl.v3d.CSG
import eu.mihosoft.vrl.v3d.Cylinder

//Your code here

CSG manip = new Cylinder(20,40).toCSG()

manip.getStorage().set("manipulator", "test")

return manip