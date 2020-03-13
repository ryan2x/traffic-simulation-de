// canvas GUI settings parameters
var userCanDistortRoads=false;
var userCanDropObjects=true;

/*######################################################
 Global overall scenario settings and graphics objects
 see onramp.js for more details

 refSizePhys  => reference size in m (generally smaller side of canvas)
 refSizePix   => reference size in pixel (generally smaller side of canvas)
 scale = refSizePix/refSizePhys 
       => roads have full canvas regardless of refSizePhys, refSizePix

 (1) refSizePix=Math.min(canvas.width, canvas.height) determined during run  

 (2) refSizePhys smaller  => all phys roadlengths smaller
  => vehicles and road widths appear bigger for a given screen size 
  => chose smaller for mobile, 

######################################################*
*/

// override standard settings control_gui.js

density=0.02;  // default 0.03
slider_density.value=1000*density;
slider_densityVal.innerHTML=1000*density+"/km";

fracTruck=0.1; // default 0.1 
slider_fracTruck.value=100*fracTruck;
slider_fracTruckVal.innerHTML=100*fracTruck+"%";


// Global overall scenario settings and graphics objects


var scenarioString="Ring";
console.log("\n\nstart main: scenarioString=",scenarioString);

var simDivWindow=document.getElementById("contents");
   // following cmd defines also mouse listeners from html 
var canvas = document.getElementById("canvas"); 
var ctx = canvas.getContext("2d"); // graphics context
canvas.width  = simDivWindow.clientWidth; 
canvas.height  = simDivWindow.clientHeight;
var aspectRatio=canvas.width/canvas.height;

console.log("before addTouchListeners()");
addTouchListeners();
console.log("after addTouchListeners()");

//##################################################################
// overall scaling (critAspectRatio should be consistent with 
// width/height in css.#contents)
//##################################################################

var isSmartphone=mqSmartphone();

var my_network = new CrossRoadNetwork(canvas, {});

// var mainroad=new road(roadID,mainroadLen,laneWidth,nLanes_main,traj_x,traj_y,
// 		      density,speedInit,fracTruck,isRing,userCanDistortRoads);

// var mainroad = my_network.network[0]; // for the moment, we will remove it soon
// network[0]=mainroad;  // network declared in canvas_gui.js
network = network.concat(my_network.network);


//  introduce stationary detectors

var detectors=[];
// for(var idet=0; idet<4; idet++){
//   detectors[idet]=new stationaryDetector(mainroad,
// 					  (0.125+idet*0.25)*mainroadLen,10);
// }
detectors = detectors.concat(my_network.detectors);



//#########################################################
// model initialization (models and methods defined in control_gui.js)
//#########################################################
	
updateModels(); // defines longModelCar,-Truck,LCModelCar,-Truck,-Mandatory


//####################################################################
// Global graphics specification
//####################################################################


// graphical settings

var hasChanged=true; // window dimensions have changed (responsive design)

var drawBackground=true; // if false, default unicolor background
var drawRoad=true; // if false, only vehicles are drawn
var userCanvasManip; //!!! true only if user-driven geometry changes


//####################################################################
// Images
//####################################################################


// init background image

var background = new Image();
background.src ='figs/backgroundGrass.jpg'; 
 

// init vehicle image(s)

carImg = new Image();
carImg.src = 'figs/blackCarCropped.gif';
truckImg = new Image();
truckImg.src = 'figs/truck1Small.png';


// init traffic light images

traffLightRedImg = new Image();
traffLightRedImg.src='figs/trafficLightRed_affine.png';
traffLightGreenImg = new Image();
traffLightGreenImg.src='figs/trafficLightGreen_affine.png';



// xxxNew 
// define obstacle images

obstacleImgNames = []; // srcFiles[0]='figs/obstacleImg.png'
obstacleImgs = []; // srcFiles[0]='figs/obstacleImg.png'
for (var i=0; i<10; i++){
  obstacleImgs[i]=new Image();
  obstacleImgs[i].src = (i==0)
    ? "figs/obstacleImg.png"
    : "figs/constructionVeh"+(i)+".png";
  obstacleImgNames[i] = obstacleImgs[i].src;
}

// xxxNew remove old def obstacle images
/*
obstacleImgs = []; // srcFiles[0]='figs/obstacleImg.png'
for (var i=0; i<10; i++){
    obstacleImgs[i]=new Image();
    obstacleImgs[i].src = (i==0)
	? 'figs/obstacleImg.png'
    : "figs/constructionVeh"+(i+0)+".png"; //!!!
}
*/

// init road images

roadImgs1 = []; // road with lane separating line
roadImgs2 = []; // road without lane separating line

for (var i=0; i<4; i++){
    roadImgs1[i]=new Image();
    roadImgs1[i].src="figs/road"+(i+1)+"lanesCropWith.png"
    roadImgs2[i]=new Image();
    roadImgs2[i].src="figs/road"+(i+1)+"lanesCropWithout.png"
}

roadImg1 = new Image();
roadImg1=roadImgs1[nLanes_main-1];
roadImg2 = new Image();
roadImg2=roadImgs2[nLanes_main-1];



//xxxNew


//############################################
// traffic objects
//############################################

// TrafficObjects(canvas,nTL,nLimit,xRelDepot,yRelDepot,nRow,nCol)
var trafficObjs=my_network.trafficObjs;


//############################################
// run-time specification and functions
//############################################

var time=0;
var itime=0;
var fps=30; // frames per second (unchanged during runtime)
var dt=timewarp/fps;


//############################################
function updateSim(){
//############################################

  // (1) update times

    time +=dt; // dt depends on timewarp slider (fps=const)
    itime++;
    isSmartphone=mqSmartphone();

    my_network.updateSim(time, dt);


  //xxxNew
  
  if(userCanDropObjects&&(!isSmartphone)&&(!trafficObjPicked)){
    trafficObjs.zoomBack();
  }


  // (6) debug output
  
  if(false){
    mainroad.writeTrucksLC();
    //mainroad.writeVehicleLCModels();

  }

}  // updateSim


//##################################################
function drawSim() {
//##################################################




    // (0) reposition physical x center coordinate as response
    // to viewport size (changes)
    // isSmartphone defined in updateSim
 

 
    var relTextsize_vmin=(isSmartphone) ? 0.03 : 0.02;
    var textsize=relTextsize_vmin*Math.min(canvas.width,canvas.height);
    //console.log("isSmartphone=",isSmartphone);

    if(false){console.log(" new total inner window dimension: ",
		window.innerWidth," X ",window.innerHeight,
		" (full hd 16:9 e.g., 1120:630)",
		" canvas: ",canvas.width," X ",canvas.height);
	     }


    if ((canvas.width!=simDivWindow.clientWidth)
	||(canvas.height != simDivWindow.clientHeight))
    {
        console.log("canvas size changed... layout needed...");
        my_network.updateDimensions(canvas);
    }

 

  // (1) update heading of all vehicles rel. to road axis
  // (for some reason, strange rotations at beginning)

    

 
  // (2) reset transform matrix and draw background
  // (only needed if changes, plus "reminders" for lazy browsers)

  ctx.setTransform(1,0,0,1,0,0);
  if(drawBackground){
    if(hasChanged||(itime<=10) || (itime%50==0) || userCanvasManip
      || (!drawRoad)){
      ctx.drawImage(background,0,0,canvas.width,canvas.height);
    }
  }

  //
  my_network.drawSim(canvas, {
      userCanvasManip,
      hasChanged,
      itime,
      scale,
  });

  // (5b) draw speedlimit-change select box

  ctx.setTransform(1,0,0,1,0,0); 
  drawSpeedlBox();
 

    // (6) draw simulated time and detector displays

    displayTime(time,textsize);
    for(var iDet=0; iDet<detectors.length; iDet++){
	detectors[iDet].display(textsize);
    }



    // (7) draw the speed colormap (text size propto widthPix

    if(drawColormap){
        displayColormap(scale*(center_xPhys-0.03*roadRadius), 
                    -scale*(center_yPhys+0.50*roadRadius), 
		    scale*35, scale*45,
		    vmin_col,vmax_col,0,100/3.6);
    }


  // may be set to true in next step if changed canvas 
  // or old sign should be wiped away 
  hasChanged=false; 

    // revert to neutral transformation at the end!
  ctx.setTransform(1,0,0,1,0,0); 

} //drawSim
 



//##################################################
// Running function of the sim thread (triggered by setInterval)
//##################################################

function main_loop() {

    updateSim();
    drawSim();
    userCanvasManip=false;
}



 //############################################
// start the simulation thread
// THIS function does all the things; everything else 
// only functions/definitions
// triggers:
// (i) automatically when loading the simulation 
// (ii) when pressing the start button defined in onramp_gui.js
//  ("myRun=setInterval(main_loop, 1000/fps);")
//############################################

console.log("first main execution");
showInfo();
var myRun=setInterval(main_loop, 1000/fps); 

