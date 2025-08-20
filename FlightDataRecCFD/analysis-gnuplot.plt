#   GNUPLOT v3.6 beta multiplot script file
#
reset
set style data lines
set size 2.0, 1.0
set origin 0.0, 0.0
set multiplot

unset key
set size 0.5,0.5
set origin 0.0,0.5
set grid
set angles degrees
#  Plot Altitude
set title "Altitude"
set xlabel "Simtime (s)"
set ylabel "Altitude (km)"
plot \
  'c:\orbiter\flightdata\DG3-reentry-3.dat' u 2:3

#  Plot Vrad
unset key
set size 0.5,0.5
set origin 0.0,0.0
set title "Radial Velocity"
set xlabel "Simtime (s)"
set ylabel "Vrad (m/s)"
plot \
  'c:\orbiter\flightdata\DG3-reentry-3.dat' u 2:7

#  Plot Vtan
unset key
set size 0.5,0.5
set origin 0.5,0.0
set title "Tangential Velocity"
set xlabel "Simtime (s)"
set ylabel "Vtan (m/s)"
plot \
  'c:\orbiter\flightdata\DG3-reentry-3.dat' u 2:8

#  Plot Arad
unset key
set size 0.5,0.5
set origin 0.5,0.5
set title "Radial Acceleration"
set xlabel "Simtime (s)"
set ylabel "Vrad (m/s^2)"
plot \
  'c:\orbiter\flightdata\DG3-reentry-3.dat' u 2:9

#  Plot Arad
unset key
set size 0.5,0.5
set origin 1.0,0.0
set title "Tangential Acceleration"
set xlabel "Simtime (s)"
set ylabel "Vtan (m/s^2)"
plot \
  'c:\orbiter\flightdata\DG3-reentry-3.dat' u 2:10

unset multiplot
#
#  Clean up: reset parameter defaults
#

reset
