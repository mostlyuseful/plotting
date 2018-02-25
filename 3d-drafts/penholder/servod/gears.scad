include <gears/gear.scad>

difference() {
    gear(hole_diameter = 2);
    translate([0,0,1])
    cylinder(h=10, d=5,  $fn=20);
}

translate([0,-10,0])
    rack(height=6);