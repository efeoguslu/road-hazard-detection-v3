
        # Set plot title and axis labels
        set title "Sensor Data Plot"
        set xlabel "Sample Index"
        set ylabel "Values"

        # Set data file
        set datafile separator ','

        # Set y-axis limits
        # set yrange [0:2]

        # Plot data from file as a line graph
        plot "iirFilterLogFile.txt" using 0:2 with lines title "iirFilterOutput", \
            "userInputLogFile.txt" using 0:2 with lines title "user", \
            "rotatedAzLogFile.txt" using 0:2 with lines title "raw"

set arrow from 5523, graph 0 to 5523, graph 1 nohead lc rgb 'red' lw 1
set arrow from 5832, graph 0 to 5832, graph 1 nohead lc rgb 'red' lw 1
set arrow from 6292, graph 0 to 6292, graph 1 nohead lc rgb 'red' lw 1
set arrow from 7882, graph 0 to 7882, graph 1 nohead lc rgb 'red' lw 1
set arrow from 9785, graph 0 to 9785, graph 1 nohead lc rgb 'red' lw 1
set arrow from 10235, graph 0 to 10235, graph 1 nohead lc rgb 'red' lw 1
set arrow from 15114, graph 0 to 15114, graph 1 nohead lc rgb 'red' lw 1
set arrow from 15676, graph 0 to 15676, graph 1 nohead lc rgb 'red' lw 1
set arrow from 15816, graph 0 to 15816, graph 1 nohead lc rgb 'red' lw 1
set arrow from 16576, graph 0 to 16576, graph 1 nohead lc rgb 'red' lw 1
set arrow from 16731, graph 0 to 16731, graph 1 nohead lc rgb 'red' lw 1
set arrow from 18180, graph 0 to 18180, graph 1 nohead lc rgb 'red' lw 1
set arrow from 18471, graph 0 to 18471, graph 1 nohead lc rgb 'red' lw 1
set arrow from 20753, graph 0 to 20753, graph 1 nohead lc rgb 'red' lw 1
set arrow from 24150, graph 0 to 24150, graph 1 nohead lc rgb 'red' lw 1
set arrow from 24396, graph 0 to 24396, graph 1 nohead lc rgb 'red' lw 1
set arrow from 27444, graph 0 to 27444, graph 1 nohead lc rgb 'red' lw 1
set arrow from 27646, graph 0 to 27646, graph 1 nohead lc rgb 'red' lw 1
set arrow from 27783, graph 0 to 27783, graph 1 nohead lc rgb 'red' lw 1
set arrow from 28757, graph 0 to 28757, graph 1 nohead lc rgb 'red' lw 1
set arrow from 29155, graph 0 to 29155, graph 1 nohead lc rgb 'red' lw 1
set arrow from 30229, graph 0 to 30229, graph 1 nohead lc rgb 'red' lw 1
set arrow from 31956, graph 0 to 31956, graph 1 nohead lc rgb 'red' lw 1
set arrow from 32208, graph 0 to 32208, graph 1 nohead lc rgb 'red' lw 1
set arrow from 32582, graph 0 to 32582, graph 1 nohead lc rgb 'red' lw 1
set arrow from 34198, graph 0 to 34198, graph 1 nohead lc rgb 'red' lw 1
set arrow from 34798, graph 0 to 34798, graph 1 nohead lc rgb 'red' lw 1
set arrow from 35235, graph 0 to 35235, graph 1 nohead lc rgb 'red' lw 1
set arrow from 35603, graph 0 to 35603, graph 1 nohead lc rgb 'red' lw 1
set arrow from 35990, graph 0 to 35990, graph 1 nohead lc rgb 'red' lw 1