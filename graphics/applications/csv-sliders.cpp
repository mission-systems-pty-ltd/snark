// Copyright (c) 2023 Mission Systems
// All rights reserved

/// @author Aspen Eyers

#include <iostream>
#include <QApplication>
#include <boost/ptr_container/ptr_vector.hpp>
#include <comma/application/command_line_options.h>
#include <comma/base/types.h>
#include <comma/csv/stream.h>
#include <comma/csv/traits.h>
#include <comma/name_value/parser.h>
#include <comma/visiting/traits.h>
#include <QThread>
#include <thread>
#include <chrono>

#include "csv_sliders/slider_gui.h"
#include "csv_sliders/slider.h"


// todo
//     ! make name fields of equal width
//     ! value field
//       ! make them of equal width
//       - turn them into text fields so that the user could type desired value by hand
//     ! --title: set main window title
//     - on input
//       - on input value move slider
//     - vertical sliders
//       - slider properties: vertical (a boolean field): if present, slider is vertical
//       --vertical: if present, all sliders are vertical by default, unless vertical=false specified
//     - slider types
//       - checkbox
//       - integer slider
//       - bar: "value" bar instead of slider
//       - dial: spin with a given extents
//       - text: just display value as text
//     - slider properties
//       - watch: rename to on-change: output on change
//       - step: float/integer sliders: values with a given step
//     - --config: current values etc
//       - save config.json with multiple sets of values
//       - load config.json
//       - save config.json
//       - a hot key/menu item to:
//         - reset all sliders to default values 
//         - print current values as json (so that uer can copy them to config.json)
//         - push current values
//         - pop current values
//         - switch between sets of pushed (or previous loaded from config) values
//     - basics for slider definition
//     - support multiple slider values
//       - load first set of values as default
//       - hot keys to toggle between sets of values
//       ? hot keys to pring current values to stderr as csv or (preferrably) json
//     ? check for "-" vs slider indices: may not be very robust
//     - layouts
//       ? qt config files
//       ? grid layout
//       ? stylesheet
// todo? check for "-" vs slider indices: may not be very robust

static void usage( bool verbose ){
    std::cerr << 
    "\nA GUI for sliders for rapid manipulation of csv-streams."
    "\n"
    "\nUsage: "
    "\n    csv-slider [<options>] <format>"
    "\n    TODO: explain the function of stdin vs no stdin mode..."
    "\n"
    "\nOptions:"
    "\n    --frequency,-f                Frequency to push the slider values to stdout, if there is no stdin (default 1Hz)"
    "\n    --gui-frequency               The frequency of the gui update in ms (default 20ms)"
    "\n    --vertical: TODO              all sliders are vertical unless explicitly specified in slider properties"
    "\n    --on-change[TODO]             output values only if they change"
    "\n"
    "\nWindow configuration"
    "\n    --window-geometry=[<x>],[<y>],[<width>],[<height>]: position of application window on screen in pixels"
    "\n                   e.g.csv-sliders --window-geometry=\"200,200,1000,20\" \"some-slider\""
    "\n        ATTENTION: due to X11 intricacies on Linux, window position is not what you think and your window"
    "\n                   may end up not where you want it; for more, see: https://doc.qt.io/qt-5/application-windows.html#window-geometry"
    "\n                   for now, find the desired window position by hand and use those window position values"
    "\n"
    "\n<format>: name-value pairs separated by semicolons"
    "\n"
    "\n    binary=[<format>]:            slider value binary output format; if not present, ascii is assumed"
    "\n                                  e.g: csv-sliders 'age;type=text;binary=s[16]'"
    "\n    default=[<value>]:            default value of the slider (if not present, min value is used)"
    "\n    format=[<format>; default=f;  unless binary specified, then default is taken from binary"
    "\n    max=<value>; default=1;       slider maximum value"
    "\n    min=<value>; default=0;       slider minimum value"
    "\n    name=<name>:                  slider name"
    "\n    type=<type>; default=slider: [TODO] slider type {slider, checkbox, text, bar?} (default slider)"
    "\n    vertical: TODO                slider is vertical (vertical=false means horizontal)"
    "\n    watch:[TODO]                  Push the value to stdout on change (default is false)"
    "\n"
    "\ncsv options" << std::endl;
    std::cerr << comma::csv::options::usage( verbose );
    std::cerr << std::endl;
    std::cerr << "\nExamples" << std::endl;
    if( verbose )
    {
        std::cerr << 
            "\n    basics that may be relevant to your actual use cases"
            "\n        draw points"
            "\n            view-points <( echo 0,0,0; echo 1000,1000,1000 )\";size=2\" \\"
            "\n                        <( csv-sliders -f 100 \"x;min=0;max=1000\" \"y;min=0;max=1000\" \"z;min=0;max=1000\" )\";size=1000\""
            "\n            view-points <( echo 0,0,0; echo 1000,1000,1000 )\";size=2\" \\"
            "\n                        <( csv-sliders -f 100 \"x;min=0;max=1000;binary=f\" \"y;min=0;max=1000;binary=f\" \"z;min=0;max=1000;binary=f\" --flush )\";size=1000;binary=3f\""
            "\n        adjust colours"
            "\n            while : ; do echo 1,2,3; sleep 1; done\\"
            "\n                | csv-sliders - \"gain;min=0;max=100;default=10\"  \"red_reduction;min=0;max=255;default=10\""
            "\n        resize and colour point cloud (rgb rounding will go away once slider type is implemented)"
            "\n            csv-sliders -f 10 'radius;min=0;max=50;default=25' \\"
            "\n                              'r;min=0;max=255;default=128' \\"
            "\n                              'g;min=0;max=255;default=128' \\"
            "\n                              'b;min=0;max=255;default=128' \\"
            "\n                | csv-eval --fields ,r,g,b 'r,g,b=round(r),round(g),round(b)' --flush \\"
            "\n                | { i=0; while IFS=, read radius r g b; do \\"
            "\n                             points-make sphere --radius=$radius --binary=3f \\"
            "\n                                 | csv-paste value=\"$i,$r,$g,$b;binary=ui,3ub\" '-;binary=3f'; (( ++i )); done \\"
            "\n                  } \\"
            "\n                | view-points '-;fields=block,r,g,b,x,y,z;binary=ui,3ub,3f'"
            "\n"
            "\n    [TODO]"
            "\n    csv-sliders '-;binary=3f' 'gain;min=0;max=1000' 'on;type=checkbox' 'age;type=text;binary=f' 'name;type=text;binary=s[16]'";
    }
    else
    {
        std::cerr << "\n    run with --help --verbose for more..." << std::endl;
    }
    std::cerr << std::endl;
    exit(0);
}

namespace snark { namespace graphics { namespace sliders {

}}} // namespace snark { namespace graphics { namespace sliders {

namespace comma { namespace visiting {

template < typename T > struct traits< snark::graphics::sliders::config< T > > {
    template < typename K, typename V > static void visit( const K&, const snark::graphics::sliders::config< T >& p, V& v ) {
        v.apply( "name", p.name );
        v.apply( "min", p.min );
        v.apply( "max", p.max );
        v.apply( "style", p.style );
        v.apply( "watch", p.watch );
        v.apply( "format", p.format );
        v.apply( "default", p.default_value );
        v.apply( "vertical", p.vertical );
    }
    template < typename K, typename V > static void visit( const K&, snark::graphics::sliders::config< T >& p, V& v ) {
        v.apply( "name", p.name );
        v.apply( "min", p.min );
        v.apply( "max", p.max );
        v.apply( "style", p.style );
        v.apply( "watch", p.watch );
        v.apply( "format", p.format );
        v.apply( "default", p.default_value );
        v.apply( "vertical", p.vertical );
    }
};

template <> struct traits< snark::graphics::sliders::input > {
    template < typename K, typename V > static void visit( const K&, const snark::graphics::sliders::input& p, V& v ) {
        v.apply( "block", p.block );
    }
    template < typename K, typename V > static void visit( const K&, snark::graphics::sliders::input& p, V& v ) {
        v.apply( "block", p.block );
    }
};

} } // namespace comma { namespace visiting {

template< typename T >
void draw_slider(QVBoxLayout& mainLayout, const snark::graphics::sliders::config<T>& format_detail, std::vector<snark::graphics::sliders::FloatSlider*>& sliders){

    QHBoxLayout* sliderLayout = new QHBoxLayout();

    QLabel* nameLabel = new QLabel(QString::fromStdString(format_detail.name));
    sliderLayout->addWidget(nameLabel);

    snark::graphics::sliders::FloatSlider* slider = new snark::graphics::sliders::FloatSlider(Qt::Horizontal);
    slider->setMinimum( format_detail.min );
    slider->setMaximum( format_detail.max );
    // slider->setRange(format_detail.min, format_detail.max);

    slider->setValue(format_detail.default_value);

    sliders.push_back(slider); // Store the pointer
    
    QLabel* valueLabel = new QLabel(QString::number(format_detail.default_value));
    QObject::connect(slider, &snark::graphics::sliders::FloatSlider::valueChanged, valueLabel, [valueLabel, slider](int value) {
        valueLabel->setText(QString::number(slider->convertValue(value)));
    });
    
    sliderLayout->addWidget(slider);
    sliderLayout->addWidget(valueLabel);

    mainLayout.addLayout(sliderLayout);
}

int main(int ac, char** av) {
    try
    {
        comma::command_line_options opts( ac, av, usage );
        comma::csv::options global_csv( opts );
        int gui_update_period_ms = opts.value< int >( "--gui-frequency", 20 );
        auto unnamed = opts.unnamed( "", "-[^;].*" );
        if( unnamed.empty() ) { COMMA_THROW( comma::exception, "You must specify an input file ('-' as stdin) or slider" ); } 

        int argc = 0;
        char** argv = nullptr;
        QApplication app(argc, argv);

        QWidget mainWindow;
        QVBoxLayout mainLayout(&mainWindow);

        auto window_geometry = comma::split_as< int >( opts.value< std::string >( "--window-geometry", ",,," ), ',', -1 );
        mainWindow.move( 
              (window_geometry[0] == -1) ? mainWindow.pos().x() : window_geometry[0]
            , (window_geometry[1] == -1) ? mainWindow.pos().y() : window_geometry[1]
        );
        // if not specified, set the window size to small & let the layout expand it
        mainWindow.resize(
              (window_geometry[2] == -1) ? 500 : window_geometry[2]
            , (window_geometry[3] == -1) ? 10 : window_geometry[3]
        ); 

        std::vector<snark::graphics::sliders::FloatSlider*> gui_sliders;

        comma::name_value::parser csv_parser( "filename", ';', '=', false );
        auto csv = csv_parser.get< comma::csv::options >( unnamed[0] );
        std::string sliders_buffer;
        std::string sliders_binary;
        std::string comma;
        uint i=0;
        if (csv.filename == "-") { 
            i=1; 
            if (unnamed.size() == 1) { COMMA_THROW( comma::exception, "You must specify at least one slider" ); }
        }
        std::vector< std::string > sliders_values( unnamed.size() ); // todo: fill from sliders so that sliders_buffer has default values        
        sliders_values={};
        snark::graphics::sliders::sliders_ptr sliders;
        sliders.reserve(unnamed.size());
        int offset = 0;
        snark::graphics::sliders::config< float > sample_config;
        sample_config.vertical = opts.exists( "--vertical" ); // todo: plug in
        for( ; i < unnamed.size(); i++ ){

            // Set up the buffer & check for binary or ascii
            auto s = csv_parser.get< comma::csv::options >( unnamed[i] );
            COMMA_ASSERT( s.binary() == csv.binary(), "expected all streams " << ( csv.binary() ? "binary" : "ascii" ) << "; got: '" << unnamed[0] << "' and '" << unnamed[i] << "'" );
            if( s.binary() ) { sliders_binary += comma + s.format().string(); comma = ","; }

            auto format_detail = comma::name_value::parser( "name", ';', '=', false ).get< snark::graphics::sliders::config< float > >( unnamed[i], sample_config );
            draw_slider<float>(mainLayout, format_detail, gui_sliders);
            auto slider = std::make_shared<snark::graphics::sliders::slider<float>>(offset, sizeof(float));
            offset += sizeof(float);
            slider->set(std::min(std::max(format_detail.default_value,format_detail.min),format_detail.max)).set_min(format_detail.min).set_max(format_detail.max);
            slider->set_name(format_detail.name);
            sliders_values.push_back( slider->as_string() );
            // sliders.at(i) = std::move(slider);
            sliders.push_back( std::move(slider) ); // todo? emplace_back() should do the same thing as push_back( std::move( slider ) )
        }

    // TODO: deal with types or formats...
    //             if(format_detail.format == "f"){
    //                 // auto s = new snark::graphics::sliders::slider<float>(0,0);
    //                 auto s = std::make_shared<snark::graphics::sliders::slider<float>>(0,0);
    //                 // auto s = reinterpret_cast<const snark::graphics::sliders::slider<float>*>(&slider);
    //                 s->set(std::min(std::max(format_detail.default_value,format_detail.min),format_detail.max)).set_min(format_detail.min).set_max(format_detail.max);
    //                 s->set_name(format_detail.name);
    //                 std::cerr << "Min is : " << s->min() << " Max is : " << s->max() << std::endl;
    //                 sliders.push_back( s );
    //             }else{ 

        sliders_buffer.resize( comma::csv::format( sliders_binary ).size() );
        mainWindow.show();

        if( csv.filename == "-" )
        {
            fd_set read_fds;
            struct timeval timeout;
            comma::csv::input_stream< snark::graphics::sliders::input > istream( std::cin, csv );
            comma::uint32 block = 0;
            bool has_block = csv.has_field( "block" );

            while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
            {
                FD_ZERO(&read_fds);
                FD_SET(STDIN_FILENO, &read_fds);

                timeout.tv_sec = 0;
                timeout.tv_usec = gui_update_period_ms * 1000;

                int ret = select(STDIN_FILENO + 1, &read_fds, NULL, NULL, &timeout);

                // Update from stdin
                if (ret > 0 && FD_ISSET(STDIN_FILENO, &read_fds)) {


                    const snark::graphics::sliders::input* p = istream.read();
                    if( !p ) { break; }
                    if( !has_block || p->block != block )
                    {
                        // todo: apply slider values on block change; fill sliders_buffer
                        // fill the buffer with dummy data
                        // for( unsigned int i = 0; i < sliders_buffer.size(); ++i ) { sliders_buffer[i] = i; }
                        //sliders_buffer = "1.0,2.2,56.8,";
                        if( csv.binary() )
                        {
                            for( unsigned int i = 0; i < sliders.size(); ++i ) { sliders[i]->write( &sliders_buffer[0] ); }
                            std::cout.write( istream.binary().last(), csv.format().size() );
                            std::cout.write( &sliders_buffer[0], sliders_buffer.size() );
                        }
                        else
                        {
                            // for( unsigned int = 0; i < sliders.size(); ++i ) { v[i] = boost::lexical_cast< std::string >( sliders[i].as_string() ); }
                        }
                        block = p->block;
                        
                    }
                    if( csv.binary() )
                    {
                        std::cout.write( istream.binary().last(), csv.format().size() );
                        std::cout.write( &sliders_buffer[0], sliders_buffer.size() );
                    }
                    else
                    {
                        std::cout << comma::join( istream.ascii().last(), global_csv.delimiter ) << global_csv.delimiter << comma::join( sliders_values, global_csv.delimiter ) << std::endl;
                    }
                    std::cout.flush();


                // Update the GUI
                } else if (ret == 0) {
                    for( unsigned int i = 0; i < sliders.size(); i++ ) { 
                        if (sliders[i]->type() == snark::graphics::sliders::slider_type::float_){
                            auto s = dynamic_cast<snark::graphics::sliders::slider<float>*>(sliders[i].get());
                            s->set(gui_sliders[i]->value());
                        }else {
                            COMMA_THROW( comma::exception, "Slider type not implemented");
                        }
                    }

                    app.processEvents();
                    sliders_values={};
                    for( unsigned int i = 0; i < sliders.size(); i++ ) { 
                        sliders_values.push_back(sliders[i]->as_string());
                    }
                } else {
                    COMMA_THROW( comma::exception, "select error");
                }
            }
        } else
        {
            // Handle case with no stdin - i.e. we just publish the slider values. 
            float frequency = opts.value< float >( "--frequency,-f", 1 );
            while(true){
                for( unsigned int i = 0; i < sliders.size(); i++ ) { 
                    if (sliders[i]->type() == snark::graphics::sliders::slider_type::float_){
                        auto s = dynamic_cast<snark::graphics::sliders::slider<float>*>(sliders[i].get());
                        s->set(gui_sliders[i]->value());
                    }else {
                        COMMA_THROW( comma::exception, "Slider type not implemented");
                    }
                }
                
                if( csv.binary() )
                {
                    for( unsigned int i = 0; i < sliders.size(); i++ ) { 
                        sliders[i]->write( &sliders_buffer[0] ); 
                    }
                    std::cout.write( &sliders_buffer[0], sliders_buffer.size() );
                }
                else{
                    sliders_values={};
                    for (const auto& slider : gui_sliders) { sliders_values.push_back(std::to_string(slider->value())); }
                    std::cout << comma::join( sliders_values, global_csv.delimiter ) << std::endl;
                }
                if( global_csv.flush ) { std::cout.flush(); }

                auto now = std::chrono::system_clock::now();
                auto loop_period = std::chrono::milliseconds(int(1000/frequency));
                while( std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - now) < loop_period ){
                    app.processEvents();
                    std::this_thread::sleep_for(std::chrono::milliseconds(gui_update_period_ms));
                }
            };
            return 0;
        }

        exit(0);
    }
    catch( std::exception& ex ) { std::cerr << "csv-slider: caught: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "csv-slider: caught unknown exception" << std::endl; }
    return 1;
}



