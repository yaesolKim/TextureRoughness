-- Make texture surface

-- r = 1.5, 2, 2.5, 3
-- r_int = 1, 2, 2, 3

r = 3
r_int = 3

arr_x = #(0)
h  = #(0)

bumpPos()

n_h = h.count
n_x = arr_x.count

makeBox()
makeSphere()


fn bumpPos =
(
	num_x = (25-r)/(0.5+2*r) * r_int
	num_y = (25-r)/(0.5+2*r)
	
	for j = 1 to num_x while j < num_x do (
		x = (0.5 + 2*r)*j/r_int
		append arr_x x
		append arr_x -x
	)
	
	for i = 1 to num_y while i < num_y do (
		h1 = (0.5 + 2*r)*i
		append h h1
		append h -h1
	)
	
)

fn makeBox =
(
    Box lengthsegs:1 widthsegs:1 heightsegs:1	length:50 width:50 height:1	pos:[0, 0, 0]	mapcoords:on	isSelected:on
	$.wirecolor = color (random 0 255) (random 0 255) (random 0 255)
  
)

fn makeSphere =
(
	for j = 1 to n_x while j <=n_x do (
		for i = 1 to n_h while i <=n_h do (
			Sphere radius:1 segments:32		scale:[1, r, 0.7]	pos:[arr_x[j], h[i],1]
			$.wirecolor = color (random 0 255) (random 0 255) (random 0 255)
			
		)
	)
)
