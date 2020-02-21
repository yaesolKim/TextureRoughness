-- Make textured surface
-- r = 1.5, 2, 2.5, 3

r = 3

arr_x = #()
arr_y = #()

bumpPos()

n_x = arr_x.count
n_y = arr_y.count

makeBox1()
makeBox2()
makeBox3()
makeBox4()

makeSphere()

fn bumpPos =
(	
	for j = 0 to 45 while j < 45 do (
		x = (0.1 + 1) + (0.2+2)*j
		--x = (0.5 + 1) + (1+2)*j
		append arr_x x
		append arr_x -x
	)
	
	for i = 0 to 16 while i <16 do (
		y = (0.1 + r) + (0.2+2*r)*i
		append arr_y y
		append arr_y -y
	)
	
)

fn makeBox1 =
(
    Box lengthsegs:1 widthsegs:1 heightsegs:1	length:100 width:100 height:1	pos:[50, 50, 0]	mapcoords:on	isSelected:on
	$.wirecolor = color (random 0 255) (random 0 255) (random 0 255)
  
)

fn makeBox2 =
(
    Box lengthsegs:1 widthsegs:1 heightsegs:1	length:100 width:100 height:1	pos:[-50, -50, 0]	mapcoords:on	isSelected:on
	$.wirecolor = color (random 0 255) (random 0 255) (random 0 255)
  
)


fn makeBox3 =
(
    Box lengthsegs:1 widthsegs:1 heightsegs:1	length:100 width:100 height:1	pos:[50, -50, 0]	mapcoords:on	isSelected:on
	$.wirecolor = color (random 0 255) (random 0 255) (random 0 255)
  
)

fn makeBox4 =
(
    Box lengthsegs:1 widthsegs:1 heightsegs:1	length:100 width:100 height:1	pos:[-50, 50, 0]	mapcoords:on	isSelected:on
	$.wirecolor = color (random 0 255) (random 0 255) (random 0 255)
  
)

fn makeSphere =
(
	for j = 1 to n_x while j <=n_x do (
		for i = 1 to n_y while i <=n_y do (
			Sphere radius:1 segments:32		scale:[1, r, 0.7]	pos:[arr_x[j], arr_y[i],1]
			$.wirecolor = color (random 0 255) (random 0 255) (random 0 255)
			
		)
	)
)