import com.neuronrobotics.bowlerstudio.creature.ICadGenerator
import com.neuronrobotics.bowlerstudio.vitamins.Vitamins
import com.neuronrobotics.sdk.addons.kinematics.AbstractLink
import com.neuronrobotics.sdk.addons.kinematics.DHLink
import com.neuronrobotics.sdk.addons.kinematics.DHParameterKinematics
import com.neuronrobotics.sdk.addons.kinematics.LinkConfiguration
import com.neuronrobotics.sdk.addons.kinematics.MobileBase
import com.neuronrobotics.sdk.addons.kinematics.math.RotationNR
import com.neuronrobotics.sdk.addons.kinematics.math.TransformNR

import eu.mihosoft.vrl.v3d.CSG
import eu.mihosoft.vrl.v3d.Cube
import eu.mihosoft.vrl.v3d.Cylinder
import eu.mihosoft.vrl.v3d.Parabola
import eu.mihosoft.vrl.v3d.Transform
import javafx.scene.transform.Affine

double boardThickness =6.3

return new ICadGenerator(){
			CSG moveDHValues(CSG incoming,DHLink dh ){
				TransformNR step = new TransformNR(dh.DhStep(0)).inverse()
				Transform move = com.neuronrobotics.bowlerstudio.physics.TransformFactory.nrToCSG(step)
				return incoming.transformed(move)
			}
			CSG reverseDHValues(CSG incoming,DHLink dh ){
				TransformNR step = new TransformNR(dh.DhStep(0))
				Transform move = com.neuronrobotics.bowlerstudio.physics.TransformFactory.nrToCSG(step)
				return incoming.transformed(move)
			}
			@Override
			public ArrayList<CSG> generateCad(DHParameterKinematics d, int linkIndex) {
				HashMap<String,Object> measurmentsMotor = Vitamins.getConfiguration(  "LewanSoulMotor","lx_224")
				double motorz =  measurmentsMotor.body_z
				double centerTheMotorsValue=motorz/2;
				TransformNR motorLocation = new TransformNR(0,0,centerTheMotorsValue,new RotationNR())

				if(linkIndex ==0) {
					ArrayList<DHLink> dhLinks = d.getChain().getLinks()
					DHLink dh = dhLinks.get(linkIndex)
					// Hardware to engineering units configuration
					LinkConfiguration  conf = d.getLinkConfiguration(linkIndex);
					// Engineering units to kinematics link (limits and hardware type abstraction)
					AbstractLink abstractLink = d.getAbstractLink(linkIndex);
					// Transform used by the UI to render the location of the object
					Affine manipulator = dh.getListener();
					//Vitamins
					def type=	d.getLinkConfiguration(linkIndex).getShaftType()
					def size = d.getLinkConfiguration(linkIndex).getShaftSize()
					CSG bearing =Vitamins.get(	d.getLinkConfiguration(linkIndex).getElectroMechanicalType(),
							d.getLinkConfiguration(linkIndex).getElectroMechanicalSize())


					def mountPlateToHornTop = 1


					CSG vitaminCad=    Vitamins.get(	type,size)
							.roty(180)
							.movez(-bearing.getTotalZ()*2)

					def baseCoreheight = 20-mountPlateToHornTop-bearing.getTotalZ()*2
					def bearingHeight =mountPlateToHornTop-2
					CSG thrust = Vitamins.get("ballBearing","Thrust_1andAHalfinch")
							.movez(bearingHeight)
					def thrustMeasurments= Vitamins.getConfiguration("ballBearing","Thrust_1andAHalfinch")
					double baseCorRad = thrustMeasurments.outerDiameter/2+5
					//END vitamins
					CSG boardBolt = Vitamins.get("capScrew","M5x25")
									.movez(boardThickness*1.5)
									.rotx(90)
									.movez(boardThickness+baseCoreheight+mountPlateToHornTop)
									.movex(25.0)
					CSG locknut = Vitamins.get("lockNut","M5")
										.movez(boardThickness*1.5)
										.rotx(-90)
										.movez(boardThickness+baseCoreheight+mountPlateToHornTop)
										.movex(25.0)
					//Mount holes
					CSG mount = Vitamins.get("heatedThreadedInsert", "M5")
							.toZMax()
							.movez(baseCoreheight+mountPlateToHornTop)
					CSG boardHole=new Cylinder(5.5/2.0,boardThickness*2).toCSG()
							.rotx(90)
							.movey(-boardThickness)
							.movez(boardThickness)
					CSG boardMountLug = new Cube( mount.getTotalX()+5, boardThickness,mount.getTotalY()+5).toCSG()
							.toZMin()
							.difference(boardHole)
							.movez(baseCoreheight+mountPlateToHornTop)
							.movex(25.0)
					def mounts =[mount]
					def lugs = []
					def vitamins=[]
					mount=mount.movex(25.0)
					for(def i=0;i<360;i+=90) {
						mounts.add(mount.rotz(i+45))
						lugs.add(boardMountLug.rotz(i))
						vitamins.add(boardBolt.rotz(i))
						vitamins.add(locknut.rotz(i))
					}
					//end Mount holes

					CSG baseCore = new Cylinder(baseCorRad,baseCorRad,baseCoreheight,36).toCSG()
							.movez(mountPlateToHornTop)
							.union(lugs)
							.difference(thrust)
							.difference(vitaminCad)
							.difference(mounts)
					mounts.addAll(vitamins)
					mounts=mounts.collect{moveDHValues(it, dh).setManipulator(manipulator)}
					vitaminCad = moveDHValues(vitaminCad, dh)
					thrust = moveDHValues(thrust, dh)
					baseCore = moveDHValues(baseCore, dh)
					//END link CAD

					baseCore.setManipulator(manipulator)
					vitaminCad.setManipulator(manipulator)
					thrust.setManipulator(manipulator)
					mounts.addAll([vitaminCad, thrust])
					
					
					for(CSG c:mounts) {
						c.setManufacturing({
							return null
						})
						c.setColor(javafx.scene.paint.Color.SILVER)
					}
					baseCore.setColor(javafx.scene.paint.Color.BLUE)
					baseCore.setManufacturing({
						return reverseDHValues(it, dh).toZMin()
					})
					baseCore.setName("craneBase")
					mounts.add(baseCore)
					baseCore.getStorage().set("printBedIndex",1);
					return mounts
				}
				return new ArrayList();
			}

			@Override
			public ArrayList<CSG> generateBody(MobileBase arg0) {
				// TODO Auto-generated method stub
				ArrayList<CSG> back =[]
//				back.add(new Cube(1).toCSG())
//				for(CSG c:back)
//					c.setManipulator(arg0.getRootListener())
//				for(DHParameterKinematics kin:arg0.getAllDHChains()) {
//					CSG limbRoot =new Cube(1).toCSG()
//					limbRoot.setManipulator(kin.getRootListener())
//					back.add(limbRoot)
//
//				}
				return back;
			}


		}