require 'mustache'

Mustache.template_path = File.dirname(__FILE__)

Mustache.template_file = File.dirname(__FILE__) + '/main.mustache'
view = Mustache.new

#---------------------------------------------------
# main body (base)
body_x = 0.1016
body_y = 0.0254
body_z = 0.6604
view[:body_mass] = 1.767
view[:body_dimensions] = "#{body_x} #{body_y} #{body_z}"

#---------------------------------------------------
# gears
view[:gear_length] = 0.01
view[:gear_rad] = 0.04
view[:gear_z_offset] = 0.01905
view[:gear_mass] = 0.1
gear_x_offset = 0.061

#---------------------------------------------------
# wheels
view[:wheel_length] = 0.0254
view[:wheel_rad] = 0.042
view[:wheel_mass] = 0.1
wheel_x_offset = 0.03298

view[:left_side] = { dir: "left", gear_x_offset: -gear_x_offset, wheel_x_offset: -wheel_x_offset, color: "red" }
view[:right_side] = { dir: "right", gear_x_offset: gear_x_offset, wheel_x_offset: wheel_x_offset, color: "green" }

path = File.dirname(__FILE__) + '/../../beaker.urdf'
open(path, 'w') { |f| f.puts view.render }