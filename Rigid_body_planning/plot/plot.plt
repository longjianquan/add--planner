#set terminal postscript eps color enhanced 20
#set output "out.eps"
set xlabel "x"
set ylabel "y"
set zlabel "z"

set key outside
set key top right
set size square
splot "path.dat" using 1:2:3 with lines lt 1 lc rgb "#191970" lw 2 title "Path",\
"path0.dat" using 1:2:3 with lines lt 1 lc rgb "#ff4500" lw 2 title "Path0"
