require 'mustache'

Mustache.template_path = File.dirname(__FILE__)
Mustache.template_file = File.dirname(__FILE__) + '/main.mustache'
view = Mustache.new

#---------------------------------------------------
# body (base)
body_x = 0.1016
body_y = 0.0254
body_z = 0.6604
view[:body_mass] = 1.767
view[:body_dimensions] = "#{body_x} #{body_y} #{body_z}"

#---------------------------------------------------
# wheels
view[:wheel_rad] = 0.042
view[:wheel_height] = 0.01905
view[:wheel_length] = 0.0254
wheel_x_offset = 0.09398

#---------------------------------------------------
# indicators
side = 0.01
view[:indicator_dimensions] = "#{side} #{side} #{side}"
view[:indicator_gear_offset] = 0.03

#---------------------------------------------------
# side differentiators
view[:left_side] = { dir: "left", wheel_x_offset: -wheel_x_offset, color: "red" }
view[:right_side] = { dir: "right", wheel_x_offset: wheel_x_offset, color: "green" }

path = File.dirname(__FILE__) + '/../../beaker.urdf'
open(path, 'w') { |f| f.puts view.render }