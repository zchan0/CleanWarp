Clean Warp
---

Usage
---

```
Usage: okwarp inimage.png [outimage.png] -f 1 | 2 (warp function) [-m 0..3](clean method)
-m 0: none clean method
-m 1: clean warp with bilinear interpolation
-m 2: clean warp with adaptive supersampling
-m 3: clean warp with both bilinear interpolation & adaptive supersampling
```

Demo
---

# With warp function 1

```
u = sqrt(x);			        
v = 0.5 * (1 + sin(y * PI)); 
```

> Before

![reconstruction](https://people.cs.clemson.edu/~dhouse/courses/404/homework/hw7/cencenz/HW%207/construction-1-before.png)

> After with bilinear interpolation

![reconstruction](https://people.cs.clemson.edu/~dhouse/courses/404/homework/hw7/cencenz/HW%207/construction-1-after.png)

# With warp function 2

```
// wave warp effect 
u = x + sin(25.0 * y + 30.0 * x + 0.3477) * 0.05;
v = y + sin(25.0 * y + 30.0 * x + 0.3477) * 0.05;
```

> Before

![patches](https://people.cs.clemson.edu/~dhouse/courses/404/homework/hw7/cencenz/HW%207/patches.png-f2-m0.png)

> After

![patches](https://people.cs.clemson.edu/~dhouse/courses/404/homework/hw7/cencenz/HW%207/patches.png-f2-m1.png)


