// Copyright (c) 2021 Vsevolod Vlaskine

// todo! add QtCharts licence

/// @author Vsevolod Vlaskine

#include <iostream>
#include <map>
#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>
#include <QtWidgets/QApplication>
#include <comma/application/command_line_options.h>
#include <comma/csv/options.h>
#include <comma/csv/traits.h>
#include <comma/string/string.h>
#include <comma/name_value/parser.h>
#include "csv_plot/main_window.h"
#include "csv_plot/traits.h"

static void usage( bool verbose = false )
{
    std::cerr << "plot points from csv files or streams" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: cat xy.csv | csv-plot [<sources>] [<options>]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "sources: input sources as <name>[;<options>]" << std::endl;
    std::cerr << "    <name>: e.g. points.csv, tcp:localhost:12345, etc" << std::endl;
    std::cerr << "    <options>: csv options, stream, and series-specific options (see below)" << std::endl;
    std::cerr << "               e.g: csv-plot \"points.bin;fields=x,y;binary=2d;color=red;weight=2\"" << std::endl;
    std::cerr << "               an option in form --xxx=<n> applies to all streams or series" << std::endl;
    std::cerr << "               or it can be defined per stream or series as 'data.csv;xxx=<n>'" << std::endl;
    std::cerr << "               e.g: csv-plot 'a.csv;fields=,,x,,y;color=red' 'b.csv' --color=blue" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --help,-h: help, --help --verbose: more help" << std::endl;
    std::cerr << "    --verbose,-v: more output" << std::endl;
    std::cerr << std::endl;
    std::cerr << "stream options" << std::endl;
    std::cerr << "    --blocking; blocking read on this stream; important for streams running from subshells" << std::endl;
    std::cerr << "                see io-cat -h -v for explanation" << std::endl;
    std::cerr << "    --input-fields; print possible input fields to stdout and exit" << std::endl;
    std::cerr << "    --input-fields-example; print input fields example to stdout and exit" << std::endl;
    std::cerr << "    --fields: t,series,block,size,number-of-series,block-by-size where series is an array; also see --number-of-series" << std::endl;
    std::cerr << "              x,y: aliases for series[0]/x and series[0]/y" << std::endl;
    std::cerr << "              series fields: x,y,z" << std::endl;
    std::cerr << "              number-of-series: see --number-of-series" << std::endl;
    std::cerr << "              size: see --size" << std::endl;
    std::cerr << "              block-by-size: fixed-size blocks, no block field required" << std::endl;
    std::cerr << "                             e.g: csv-plot '-;fields=x,y;size=256,block-by-size'" << std::endl;
    std::cerr << "                                  also see histogram example below" << std::endl;
    std::cerr << "              default: x,y" << std::endl;
    std::cerr << "              if x is not present in series n, x will be same as in series 0" << std::endl;
    std::cerr << "              examples" << std::endl;
    std::cerr << "                  --fields=series[0]/x,series[0]/y,series[1]/x,series[1]/y" << std::endl;
    std::cerr << "                  --fields=series[0],series[1]: means same as above" << std::endl;
    std::cerr << "                  --fields=x,y,series[1]: means same as above" << std::endl;
    std::cerr << "                  --fields=series[0],series[1]/y,series[2]/y: series 1 and 2 take x from series 0" << std::endl;
    std::cerr << "    --no-stdin: don't try to read from stdin" << std::endl;
    std::cerr << "    --number-of-series,-n=<n>; default=1; how many series each stream has; a convenience option" << std::endl;
    std::cerr << "                               if --fields have 'series' field without series indices" << std::endl;
    std::cerr << "    --pass-through,--pass; todo: output to stdout the first stream on the command line" << std::endl;
    std::cerr << "    --size,-s,--tail=<n>: plot last <n> records of stream; default: file: estimated number of records" << std::endl;
    std::cerr << "                                                                    stream: 10000 records" << std::endl;
    std::cerr << std::endl;
    std::cerr << std::endl << comma::csv::options::usage( verbose ) << std::endl;
    std::cerr << std::endl;
    std::cerr << "chart options" << std::endl;
    std::cerr << "    --chart=<name>,<properties>; semicolon-separated chart properties; multiple --chart options allowed" << std::endl;
    std::cerr << "        <properties>" << std::endl;
    std::cerr << "            animate; default: true" << std::endl;
    std::cerr << "            axes/x/label/format=[<format>]; <format>: whatever format printf() takes (see examples)" << std::endl;
    std::cerr << "            axes/x/tick/anchor=[<anchor>]; default: use min/x if defined or auto if not" << std::endl;
    std::cerr << "            axes/x/tick/count=[<n>]; desired number of ticks" << std::endl;
    std::cerr << "            axes/x/tick/interval=[<interval>]; default: auto if not defined" << std::endl;
    std::cerr << "            axes/x/title=[<title>]" << std::endl;
    std::cerr << "            axes/y/label/format=[<format>]; <format>: whatever format printf() takes (see examples)" << std::endl;
    std::cerr << "            axes/y/tick/anchor=[<anchor>]; default: use min/y if defined or auto if not" << std::endl;
    std::cerr << "            axes/y/tick/count=[<n>]; desired number of ticks" << std::endl;
    std::cerr << "            axes/y/tick/interval=[<interval>]; default: auto if not defined" << std::endl;
    std::cerr << "            axes/y/title=[<title>]" << std::endl;
    std::cerr << "            legend; show legend; default: false" << std::endl;
    std::cerr << "            max/x=[<value>], max/y=[<value>]" << std::endl;
    std::cerr << "            min/x=[<value>], min/y=[<value>]" << std::endl;
    std::cerr << "            scroll; \"scroll\" to the current chart extents" << std::endl;
    std::cerr << "            title=[<title>]; default: <name>" << std::endl;
    std::cerr << "            theme=[<theme>]; default=light; choices: light, blue-cerulean, dark, brown-sand, blue-ncs, high-contrast, blue-icy, qt" << std::endl;
    std::cerr << "        example: todo" << std::endl;
    std::cerr << "    --scroll: if present, chart axes get adjusted to where the data is" << std::endl;
    std::cerr << std::endl;
    std::cerr << "series options" << std::endl;
    std::cerr << "    there are three equivalent ways to define series properties (the choice is determined by what looks better on specific command line)" << std::endl;
    std::cerr << "        - using command line option like --color or --shape (see below)" << std::endl;
    std::cerr << "        - using --series option" << std::endl;
    std::cerr << "        - or as series properties of a stream" << std::endl;
    std::cerr << "    examples" << std::endl;
    std::cerr << "        csv-plot --shape=spline --color=red" << std::endl;
    std::cerr << "        csv-plot '-;shape=spline;color=red'" << std::endl;
    std::cerr << "        csv-plot '-;series[0]=shape:spline|color:red'" << std::endl;
    std::cerr << "        csv-plot '-;series[0]=name:xxx' --series='xxx;shape=spline;color=red'" << std::endl;
    std::cerr << "    --series=<name>,<properties>; semicolon-separated chart properties; multiple --series options allowed" << std::endl;
    std::cerr << "        <properties>" << std::endl;
    std::cerr << "            todo" << std::endl;
    std::cerr << "    --color=<color>: plot color: black, white, red, green, blue" << std::endl;
    std::cerr << "                                 yellow, cyan, magenta, grey" << std::endl;
    std::cerr << "                                 or #rrggbb, e.g. #ff00ff" << std::endl;
    std::cerr << "    --colors,--colours=<colors>: cyclic colour map name or comma-separated list of colours" << std::endl;
    std::cerr << "        if series colour is defined explicitly, it overrides the colour map entry" << std::endl;
    std::cerr << "        map choices: basic-16, rgb, cmyk, todo! more colour maps" << std::endl;
    std::cerr << "        examples" << std::endl;
    std::cerr << "            --colours=rgb" << std::endl;
    std::cerr << "            --colours=blue,green,red,cyan,magenta" << std::endl;
    std::cerr << "    --shape=<what>; <what>: line (default), scatter, spline" << std::endl;
    std::cerr << "                    todo: more shapes" << std::endl;
    std::cerr << "    --title=[<title>]: series title" << std::endl;
    std::cerr << "    --weight=<weight>: point or line weight" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    for multiple series per stream, individual series options look like: 'series[2]=color:green|chart:test'" << std::endl;
    std::cerr << "    i.e. options are |-separated <name>:<value> pairs, for example, in the following command line:" << std::endl;
    std::cerr << "        csv-plot '-;fields=series;number-of-series=4;color=red;chart=one;series[1]=color:blue;series[2]=color:green|chart:two'" << std::endl;
    std::cerr << "        - there are 4 series on stdin with fields: series[0]/x,series[0]/y,series[1]/x,series[1]/y,series[2]/x,series[2]/y,series[3]/x,series[3]/y" << std::endl;
    std::cerr << "        - series 2 will be shown in chart 'two'; series 0, 1, and 3 in chart 'one'" << std::endl;
    std::cerr << "        - series 1 will be blue; series 2 green; series 0 and 3 red" << std::endl;
    std::cerr << std::endl;
    std::cerr << "on-exit options" << std::endl;
    std::cerr << "    --on-exit=<options>" << std::endl;
    std::cerr << "        <options>" << std::endl;
    std::cerr << "            capture[=<filename>]: capture last view on exit, e.g:" << std::endl;
    std::cerr << "                                  --on-exit=capture (image name will be system time)" << std::endl;
    std::cerr << "                                  --on-exit=capture=last-view.png" << std::endl;
    std::cerr << std::endl;
    std::cerr << "window options" << std::endl;
    std::cerr << "    --frames-per-second,--fps=<value>; default=10; how often to update chart(s)" << std::endl;
    std::cerr << "    --full-screen,--maximize: todo: initially, create full screen windows" << std::endl;
    std::cerr << "    --layout=<layout>; default=grid; layouts for multiple charts" << std::endl;
    std::cerr << "        <layout>" << std::endl;
    std::cerr << "            grid[;<shape>]: charts are arranged in single window as grid" << std::endl;
    std::cerr << "                <shape>: default: 1" << std::endl;
    std::cerr << "                    cols=<cols>: grid with <cols> columns (rows calculated from number of charts)" << std::endl;
    std::cerr << "                    cols-stretch=<factors>; if present, factors are integer and number of factors is same as" << std::endl;
    std::cerr << "                                            number of grid columns; e.g. if col-stretch=30,50,20" << std::endl;
    std::cerr << "                                            then columns will take width of 30, 50, and 20 per cent" << std::endl;
    std::cerr << "                                            respectively (factors don't need to represent percentage" << std::endl;
    std::cerr << "                                            values; col-stretch=3,5,2 will have the same effect)" << std::endl;
    std::cerr << "                    rows=<rows>: grid with <rows> rows (columns calculated from number of charts)" << std::endl;
    std::cerr << "                    rows-stretch=<factors>; same as cols-stretch, but for rows" << std::endl;
    std::cerr << "                    default: cols=1" << std::endl;
    std::cerr << "                stacked: todo: each chart is in its own window" << std::endl;
    std::cerr << "                tabs: charts are arranged in single window as tabs" << std::endl;
    std::cerr << "                windows: todo: each chart is in its own window" << std::endl;
    std::cerr << "    --timeout=<seconds>; how often to update, overrides --fps" << std::endl;
    std::cerr << "    --window-geometry,--window-position,--window=[<x>,<y>[,<width>,<height>]]: position of application window on screen in pixels" << std::endl;
    std::cerr << "        ATTENTION: due to X11 intricacies on Linux, window position is not what you think and your window" << std::endl;
    std::cerr << "                   may end up not where you want it; for more, see: https://doc.qt.io/qt-5/application-windows.html#window-geometry" << std::endl;
    std::cerr << "                   for now, find the desired window position by hand and use those window position values" << std::endl;
    std::cerr << "    --window-size=<x>,<y>: initial window size, same as <width>,<height> in --window-position; default=800,600" << std::endl;
    std::cerr << std::endl;
    std::cerr << "zooming and panning" << std::endl;
    std::cerr << "    scroll wheel: zoom in and out about the mouse cursor" << std::endl;
    std::cerr << "    left-click and drag: zoom into selected area" << std::endl;
    std::cerr << "    right-click and drag: zoom out about the centre of the selected area" << std::endl;
    std::cerr << "    +: zoom in about the centre of the plot" << std::endl;
    std::cerr << "    -: zoom out about the centre of the plot" << std::endl;
    std::cerr << "    r: reset the plot to its original zoom level" << std::endl;
    std::cerr << "    arrow keys: pan" << std::endl;
    std::cerr << "    middle-click and drag: pan" << std::endl;
    std::cerr << std::endl;
    std::cerr << "hot keys" << std::endl;
    std::cerr << "    ctrl-g : print window geometry as <left>,<top>,<width>,<height> to stderr" << std::endl;
    std::cerr << "    p      : capture current view to <timestamp>.png file" << std::endl;
    std::cerr << "    <ESC>  : exit" << std::endl;
    std::cerr << std::endl;
    if( verbose )
    {
        std::cerr << "examples" << std::endl;
        std::cerr << "    plot points on stdin" << std::endl;
        std::cerr << "        echo -e 1,10\\\\n2,5\\\\n3,8 | csv-plot" << std::endl;
        std::cerr << "        echo -e 1,10\\\\n2,5\\\\n3,8 | csv-plot --color=red" << std::endl;
        std::cerr << "        echo -e 1,10\\\\n2,5\\\\n3,8 | csv-plot --color=red --weight=5" << std::endl;
        //std::cerr << "        echo -e 1,10\\\\n2,5\\\\n3,8 | csv-plot --color=red --weight=5 --style=dots" << std::endl;
        std::cerr << std::endl;
        std::cerr << "    use point number as x" << std::endl;
        std::cerr << "        echo -e 10\\\\n5\\\\n8 | csv-plot --fields=y" << std::endl;
        std::cerr << std::endl;
        std::cerr << "    use point number as x" << std::endl;
        std::cerr << "        echo -e 10\\\\n5\\\\n8 | csv-plot --fields y" << std::endl;
        std::cerr << std::endl;
        std::cerr << "    several plots at once" << std::endl;
        std::cerr << "        echo -e 1,10\\\\n2,5\\\\n3,8 > test.csv" << std::endl;
        std::cerr << "        csv-plot \"test.csv;color=red\" \"test.csv;color=blue;fields=y,x\"" << std::endl;
        std::cerr << std::endl;
        std::cerr << "    specify tick properties for x axis (same for y axis)" << std::endl;
        std::cerr << "        ( echo -1,10; echo 0.5,15; echo 2,3 ) \\" << std::endl;
        std::cerr << "            | csv-plot --chart ';min/x=-2;max/x=2;axes/tick/anchor=0;axes/x/tick/interval=0.3'" << std::endl;
        std::cerr << std::endl;
        std::cerr << "    specify label format for x and y axes" << std::endl;
        std::cerr << "        ( echo 0.1,0.3; echo 0.9,2.9; echo 4.85,7.1; echo 8.93,8.7 ) \\" << std::endl;
        std::cerr << "            | csv-plot --chart ';min/x=0;max/x=10;axes/x/tick/interval=2;axes/y/tick/interval=1;axes/x/label/format=\%d pretzels;axes/y/label/format=$\%2.0f'" << std::endl;
        std::cerr << "    point streams: show last 100 points" << std::endl;
        std::cerr << "        netcat localhost 12345 | csv-plot --size=100" << std::endl;
        std::cerr << std::endl;
        std::cerr << "    specify chart theme" << std::endl;
        std::cerr << "        csv-paste line-number --head 1000 | csv-eval --fields=x 'y=8*sin(x/10)' | csv-plot '-;color=yellow' --chart ';theme=dark'" << std::endl;
        std::cerr << std::endl;
        std::cerr << "    several streams" << std::endl;
        std::cerr << "        csv-plot \"tcp:localhost:8888\" \"tcp:localhost:9999\"" << std::endl;
        std::cerr << std::endl;
        std::cerr << "    plot block by block" << std::endl;
        std::cerr << "        netcat localhost 12345 | csv-plot --fields=x,y,block" << std::endl;
        std::cerr << std::endl;
        std::cerr << "    xy chart with multiple inputs block by block with different shapes" << std::endl;
        std::cerr << "        - all plots on the same chart" << std::endl;
        std::cerr << "            csv-random make --type f --range=0,20 | csv-paste 'line-number;size=10' 'line-number;size=10;index' - \\" << std::endl;
        std::cerr << "                | csv-plot '-;fields=block,x,y;color=red;weight=2' \\" << std::endl;
        std::cerr << "                           <( csv-random make --type f --range=0,20 | csv-paste 'line-number;size=10' 'line-number;size=10;index' - )';fields=block,x,y;color=blue;weight=2;shape=spline' \\" << std::endl;
        std::cerr << "                           <( csv-random make --type f --range=0,20 | csv-paste 'line-number;size=10' 'line-number;size=10;index' - )';fields=block,x,y;color=green;weight=5;shape=scatter'" << std::endl;
        std::cerr << "        - plots on different charts with grid layout: same command as above, but add 'chart=...'" << std::endl;
        std::cerr << "            - default layout: charts are stacked in a single column" << std::endl;
        std::cerr << "                ... | csv-plot ... '...;fields=...;chart=A' '...;fields=...;chart=B' '...;fields=...;chart=C'" << std::endl;
        std::cerr << "            - two columns:" << std::endl;
        std::cerr << "                ... | csv-plot --layout='grid;cols=2' ... '...;fields=...;chart=A' '...;fields=...;chart=B' '...;fields=...;chart=C'" << std::endl;
        std::cerr << "            - one row:" << std::endl;
        std::cerr << "                ... | csv-plot --layout='grid;rows=1' ... '...;fields=...;chart=A' '...;fields=...;chart=B' '...;fields=...;chart=C'" << std::endl;
        std::cerr << std::endl;
        std::cerr << "    xy chart with sliding window on endless series with 2 frames per second refresh, using --scroll" << std::endl;
        std::cerr << "        csv-random make --type f --range=0,20 | csv-paste line-number - | csv-repeat --pace --period 0.1 \\" << std::endl;
        std::cerr << "            | csv-plot --fps 2 --scroll '-;color=red;weight=2;size=40' \\" << std::endl;
        std::cerr << "                       <( csv-random make --seed 1234 --type f --range=0,30 | csv-paste line-number - | csv-repeat --pace --period 0.1 )';color=blue;weight=2;size=50'" << std::endl;
        std::cerr << std::endl;
        std::cerr << "    multiple series per stream" << std::endl;
        std::cerr << "        basics" << std::endl;
        std::cerr << "            csv-random make --type 4f --range=0,20 \\" << std::endl;
        std::cerr << "                | csv-paste 'line-number;size=10' 'line-number;size=10;index' - \\" << std::endl;
        std::cerr << "                | csv-plot '-;fields=block,x,y,series[0]/y,series[1]/y,series[2]/y' --fps 1" << std::endl;
        std::cerr << "        individual series options" << std::endl;
        std::cerr << "            csv-random make --type 4f --range=0,20 \\" << std::endl;
        std::cerr << "                | csv-paste 'line-number;size=10' 'line-number;size=10;index' - \\" << std::endl;
        std::cerr << "                | csv-plot '-;fields=block,x,y,series[0]/y,series[1]/y,series[2]/y;series[1]=color:blue;series[2]=color:green|chart:test2' --fps 1" << std::endl;
        std::cerr << std::endl;
        std::cerr << "    display image histogram from the laptop camera" << std::endl;
        std::cerr << "        cv-cat --camera 'resize=640,480;view' \\" << std::endl;
        std::cerr << "            | cv-calc histogram --interleave --output no-header \\" << std::endl;
        std::cerr << "            | csv-paste 'line-number;size=256;index;binary=ui' \\" << std::endl;
        std::cerr << "                        '-;binary=3ui' \\" << std::endl;
        std::cerr << "            | csv-plot '-;fields=x,series[0]/y,series[1]/y,series[2]/y;binary=4ui;series[0]=color:blue|title:blue;series[1]=color:green|title:green;series[2]=color:red|title:red;chart=histogram;size=256;block-by-size' \\" << std::endl;
        std::cerr << "                       --chart='histogram;min/y=0;max/y=8000;axes/x/title=pixel value;axes/y/title=number of pixels'" << std::endl;
        std::cerr << std::endl;
        std::cerr << "   use predefined and user-defined color maps" << std::endl;
        std::cerr << "       csv-random make --type=f \\" << std::endl;
        std::cerr << "           | csv-paste 'line-number;size=16' -  \\" << std::endl;
        std::cerr << "           | csv-shape concatenate --size 16  \\" << std::endl;
        std::cerr << "           | head -n100 \\" << std::endl;
        std::cerr << "           | csv-plot '-;fields=series' --chart=';max/y=1' -n 16 --colors=basic-16 --pass-through \\" << std::endl;
        std::cerr << "           | csv-plot '-;fields=series' --chart=';max/y=1' -n 16 --colors=red,blue,cyan,#F123456" << std::endl;
        std::cerr << "           " << std::endl;
    }
    else
    {
        std::cerr << "examples: use --help --verbose..." << std::endl;
    }
    std::cerr << std::endl;
    exit( 0 );
}

// todo
// ! --span additionally to --size
// ? performance: struggles with more than 10000 points; find bottlenecks (currently, lots of non-zero copies of buffers
// ! gitlab: tutorial
// ! application/examples/csv-plot/...: example command lines
// ? extents -> separate generic class
// - examples
//   - colour wheel in polar
// - cmake: turn on by default
// - input
//   - t
//     - as x axis (QtCharts::QDateTimeAxis?)
//     ? or simply if x is optional and t not emptys
//       - optionally offset by the first timestamp
//   - label
//     - optional input field
//     ? time as label (instead of QtCharts::QDateTimeAxis)
//   ? optional individual color for each input point
// - pan and zoom
//   - zoom
//     - on mouse wheel
//     - on rectangle selection
//     ? zoom buttons?
//   - pan
//     ? on mouse events?
//     ? pan buttons?
// - save as
//   - png
//   ? save all charts
// - chart
//   - properties
//     ? optionally show block number? optional block label field (in streams)?
//   - types
//     - time series chart
//     ? polar charts
//     ? pie chart
//     ? bar chart
//     ? 2.5d charts (not available in qtcharts)
//       ? use q3surface? qt3dvisualisation?
//       ? use background image?
//   ? optionally show grid
//   - axes
//     - handle range of zero length
//     - check range validity
//     ? add configurable margins
//     - axis types (e.g. int)
//     - configurable ticks
//     - extents policies
//       - auto-adjust
//         ? optional downscaling
//         ? better autoscaling
//         ? better autoscrolling
//       ? ignore outliers? e.g. percentile-based?
//       - t markers on x axis (QtCharts::QDateTimeAxis?)
// - series properties
//   ? optionally: series[2]=<name>
//   - properties
//     - as policy templated on qt series?
//   - scatter: style
//     ? marker shape
//   ? --scroll: separate logic from chart --scroll or remove from series altogether?
// - layouts
//   - multi-window
//   ? stacked
// - main window
//   - add signal to update? currently, updates only after first timeout
// - building
//   ? packaging as a separate package
//       ? move into a separate repository or add a separate cmake for cpack packaging
//       ? copy-paste block_buffer
//       ? package
//       ? expose on ppa
// ! qtcharts licence
// ! don't use block buffer as is? use double-buffered QList and pop front if exceeds size? (so far performance looks ok)
// - qml
//   ? qml import
//   ? qml export
//   ? totally rewrite everything using qml?

// todo for Hamish
// - fix: run histogram demo, try to zoom in and then zoom out - x axis gets squashed
//        and the only way to restory is pressing 'r'
//   - this can be mitigated by adding min and max values for the x axis as well as the y axis
//     - try without any min values for x or y -> no zooming possible at all
// - help:
//   ? help menu or status line at the bottom with prompt (maybe it's too much too early)

QT_USE_NAMESPACE

template < typename P > static std::map< std::string, P > make_configs( const std::vector< std::string >& properties, const P& defaults = P() )
{
    std::map< std::string, P > m;
    for( const auto& p: properties )
    {
        auto c = comma::name_value::parser( "name", ';', '=', true ).get< P >( p, defaults );
        if( c.title.empty() ) { c.title = c.name; } // quick and dirty
        m[ c.name ] = c;
    }
    return m;
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        bool verbose = options.exists( "--verbose,-v" );
        if( options.exists( "--input-fields" ) ) { std::cout << "t,series,block" << std::endl; return 0; } // quick and dirty
        if( options.exists( "--input-fields-example" ) ) { std::cout << comma::join( comma::csv::names< snark::graphics::plotting::record >( true, snark::graphics::plotting::record::sample( "series", 2 ) ), ',' ) << std::endl; return 0; }
        const std::vector< std::string >& unnamed = options.unnamed( "--blocking,--no-stdin,--verbose,-v,--flush,--full-screen,--maximize,--pass-through,--pass,--scroll", "--.*,-[a-z].*" );
        boost::optional< unsigned int > stdin_index = boost::make_optional< unsigned int >( false, 0 );
        for( unsigned int i = 0; i < unnamed.size(); ++i ) { if( unnamed[i] == "-" || unnamed[i].substr( 0, 2 ) == "-;" ) { stdin_index = i; break; } }
        const auto& chart_configs = make_configs( options.values< std::string >( "--chart" ), snark::graphics::plotting::chart::config_t( options ) );
        const auto& series_configs = make_configs( options.values< std::string >( "--series" ), snark::graphics::plotting::series::config( options ) );
        snark::graphics::plotting::stream::config_t stream_config( options );
        std::vector< snark::graphics::plotting::stream::config_t > stream_configs;
        bool use_stdin = !options.exists( "--no-stdin" );
        COMMA_ASSERT_BRIEF( use_stdin || !stdin_index, "due to --no-stdin, expected no stdin options; got: \"" << unnamed[ *stdin_index ] << "\"" );
        if( use_stdin && !stdin_index )
        {
            stream_config.csv.filename = "-";
            auto config = stream_config;
            if( config.size == 0 ) { config.size = 10000; }
            stream_configs.push_back( config );
            stream_config.pass_through = false;
        }
        comma::saymore() << "got " << stream_configs.size() << " input stream config(s)" << std::endl;
        for( const auto& u: unnamed ) { stream_configs.push_back( snark::graphics::plotting::stream::config_t( u, series_configs, stream_config ) ); stream_config.pass_through = false; }
        comma::saymore() << "got " << stream_configs.size() << " input stream config(s)" << std::endl;
        float timeout = options.value( "--timeout", 1. / options.value( "--frames-per-second,--fps", 10 ) );
        std::string layout = options.value< std::string >( "--layout", "grid" );
        std::string on_exit_options = options.value< std::string >( "--on-exit", "" );
        QApplication application( ac, av );
        snark::graphics::plotting::main_window main_window( application, stream_configs, chart_configs, layout, timeout, on_exit_options );
        auto window_size = comma::csv::ascii< std::pair< unsigned int, unsigned int > >().get( options.value< std::string >( "--window-size", "800,600" ) );
        std::string window_position = options.value< std::string >( "--window-geometry,--window-position,--window", "" );
        if( window_position.empty() )
        {
            main_window.move( 0, 0 );
        }
        else
        {
            const auto& p = comma::split_as< unsigned int >( window_position, ',' );
            if( p.size() != 2 && p.size() != 4 ) { comma::say() << "expected --window-geometry=<x>,<y>[,<width>,<height>]; got: \"" << window_position << "\"" << std::endl; return 1; }
            main_window.move( p[0], p[1] );
            if( p.size() == 4 ) { window_size = { p[2], p[3] }; }
        }
        main_window.resize( window_size.first, window_size.second );
        if( verbose )
        {
            comma::saymore() << "created " << main_window.charts().size() << " chart(s)" << std::endl;
            comma::saymore() << "created " << main_window.streams().size() << " input stream(s)" << std::endl;
            for( unsigned int i = 0; i < main_window.streams().size(); ++i )
            {
                for( unsigned int j = 0; j < main_window.streams()[i].series.size(); ++j ) { comma::say() << "stream " << i << ": series " << j << " will be shown on " << ( main_window.streams()[i].series[j].config().chart.empty() ? std::string( "unnamed chart" ) : ( "chart named: '" + main_window.streams()[i].series[j].config().chart + "'" ) ) << std::endl; }
            }
            if( !main_window.pass_through_stream_name().empty() ) { comma::say() << "stream '" << main_window.pass_through_stream_name() << "' will be passed through" << std::endl; }
        }
        main_window.start();
        options.exists( "--full-screen,--maximize" ) ? main_window.showMaximized() : main_window.show();
        comma::saymore() << "started" << std::endl;
        comma::saymore() << "hotkeys" << std::endl;
        comma::saymore() << "    ctrl+g : print window geometry to stderr" << std::endl;
        comma::saymore() << "    p      : capture current view to <timestamp>.png file" << std::endl;
        comma::saymore() << "    <ESC>  : exit" << std::endl;
        return application.exec();
    }
    catch( std::exception& ex ) { std::cerr << "csv-plot: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "csv-plot: unknown exception" << std::endl; }
    return 1;
}
