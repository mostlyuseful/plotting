include <gears/gear.scad>
pi = PI;

// Eckdaten Zahnrad
// 3.0 -> zu eng
// 4.0 -> bisschen zu weit
// 3.5 -> zu eng
// 3.8 -> perfekt
r_outer = 3.6; // mm
number_of_teeth = 14;
mm_per_tooth = 2*pi*r_outer / (number_of_teeth+2);
height_inner = 7;
height_outer = 2;
dalpha = 360/5;

//intersection() {    
    //cylinder(h=100,d=15, center=true);

union(){
difference() {

union(){
cylinder(h=height_inner, d=11, center=false);
    color([0.5,0.5,1])
cylinder(h=height_outer, d=45, center=false);
}
    
union() {
    
    translate([0,0,-10])
    gear(
        mm_per_tooth = mm_per_tooth,
        hole_diameter = 0, 
        number_of_teeth = number_of_teeth,
        pressure_angle = 28,
        thickness = 10*height_inner
);

}

}

color([1,0,0])
translate([0,0,height_inner+0.5])
cylinder(h=1, d=11, center=true);

for(i=[0:4])
    rotate(a=dalpha * i, v=[0,0,1])
        translate([15,0,9/2+height_outer])
            difference(){
                cylinder(h=10, d=13, center=true);
                cylinder(h=29, d=12, center=true);
            }

translate([21,-1.5,0])
    cube([6,3,1], center=false);

}
//}