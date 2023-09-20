#include <iostream>
#include <QApplication>
#include <boost/ptr_container/ptr_vector.hpp>
#include <comma/application/command_line_options.h>
#include <comma/base/types.h>
#include <comma/csv/stream.h>
#include <comma/csv/traits.h>
#include <comma/name_value/parser.h>
#include <comma/visiting/traits.h>
#include "csv_sliders/sliderwindow.h"
#include "csv_sliders/slider.h"
#include <QThread>
#include <thread>
#include <chrono>

static void usage( bool verbose ){
    std::cerr << 
    "\nA GUI for sliders for rapid manipulation of csv-streams."
    "\n"
    "\nUsage: "
    "\n    csv-slider [options] [<format>]"
    "\n"
    "\nOptions:"
    "\n    --binary,-b:         Input binary data."
    "\n"
    "\n    --help|-h:           Display this help message"
    "\n"
    "\n    --frequency,-f       Frequenchy to push the slider values to stdout, if there is no stdin (default 1Hz)"
    "\n"
    "\n    --gui-freq           The frequency of the gui update in ms (default 20ms)"
    "\n"
    "\nFormat:"
    "\n"
    "\n    name:                The name of the slider"
    "\n"
    "\n    min:                 The slider minimum value"
    "\n"
    "\n    max:                 The slider maximum value"
    "\n"
    "\n    binary:[TODO]        The binary format of the slider output data"
    "\n                         e.g. csv-sliders 'age;type=text;binary=s[16]'"
    "\n"
    "\n    type:[TODO]          The type of the slider {slider, checkbox, text} (default slider)"
    "\n"
    "\n    watch:[TODO]         Push the value to stdout on change (default is false)"
    "\n"
    "\n    format:              The format of the slider value (e.g. f, d, ui, i, etc.)"
    "\n"
    "\n    default_value:       The default value of the slider (default is min)"
    "\n"
    "\nExample:"
    "\n    TODO: this shows the requirement for floating point values!"
    "\n"
    "\n    view-points <( echo 0,0,0; echo 1000,1000,1000 )\";size=2\" \\"
    "\n                <( ./bin/csv-sliders -f 100 \"x;min=0;max=1000\" \"y;min=0;max=1000\" \"z;min=0;max=1000\" )\";size=1000\""
    "\n"
    "\n    view-points <( echo 0,0,0; echo 1000,1000,1000 )\";size=2\" \\"
    "\n                <( ./bin/csv-sliders -f 100 \"x;min=0;max=1000;binary=f\" \"y;min=0;max=1000;binary=f\" \"z;min=0;max=1000;binary=f\" --flush )\";size=1000;binary=3f\""
    "\n"
    "\n    while : ; do echo 1,2,3; sleep 1; done\\"
    "\n         | ./bin/csv-sliders - \"gain;min=0;max=100;default_value=10\"  \"red_reduction;min=0;max=255;default_value=10\""
    "\n"
    "\n    [TODO]"
    "\n    csv-sliders '-;binary=3f' 'gain;min=0;max=1000' 'on;type=checkbox' 'age;type=text;binary=f' 'name;type=text;binary=s[16]'"

    << std::endl;
    exit(0);
}

namespace snark { namespace sliders {


} } // namespace snark { namespace sliders {



namespace comma { namespace visiting {

template < typename T > struct traits< snark::sliders::config< T > > {
    template < typename K, typename V > static void visit( const K&, const snark::sliders::config< T >& p, V& v ) {
        v.apply( "name", p.name );
        v.apply( "min", p.min );
        v.apply( "max", p.max );
        v.apply( "style", p.style );
        v.apply( "watch", p.watch );
        v.apply( "format", p.format );
        v.apply( "default_value", p.default_value );
    }
    template < typename K, typename V > static void visit( const K&, snark::sliders::config< T >& p, V& v ) {
        v.apply( "name", p.name );
        v.apply( "min", p.min );
        v.apply( "max", p.max );
        v.apply( "style", p.style );
        v.apply( "watch", p.watch );
        v.apply( "format", p.format );
        v.apply( "default_value", p.default_value );
    }
};

template <> struct traits< snark::sliders::input > {
    template < typename K, typename V > static void visit( const K&, const snark::sliders::input& p, V& v ) {
        v.apply( "block", p.block );
    }
    template < typename K, typename V > static void visit( const K&, snark::sliders::input& p, V& v ) {
        v.apply( "block", p.block );
    }
};

} } // namespace comma { namespace visiting {


template< typename T >
void draw_slider(QVBoxLayout& mainLayout, const snark::sliders::config<T>& format_detail, std::vector<FloatSlider*>& sliders){

    QHBoxLayout* sliderLayout = new QHBoxLayout();

    QLabel* nameLabel = new QLabel(QString::fromStdString(format_detail.name));
    sliderLayout->addWidget(nameLabel);

    FloatSlider* slider = new FloatSlider(Qt::Horizontal);
    slider->setMinimum( format_detail.min );
    slider->setMaximum( format_detail.max );
    // slider->setRange(format_detail.min, format_detail.max);

    slider->setValue(format_detail.default_value);

    sliders.push_back(slider); // Store the pointer
    
    QLabel* valueLabel = new QLabel(QString::number(format_detail.default_value));
    QObject::connect(slider, &FloatSlider::valueChanged, valueLabel, [valueLabel, slider](int value) {
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
        int gui_update_period_ms = opts.value< int >( "--gui-freq", 20 );
        auto unnamed = opts.unnamed( "", "-[^;].*" );
        if( unnamed.empty() ) { COMMA_THROW( comma::exception, "You must specify an input file" ); } 

        int argc = 0;
        char** argv = nullptr;
        QApplication app(argc, argv);

        QWidget mainWindow;
        QVBoxLayout mainLayout(&mainWindow);
        std::vector<FloatSlider*> gui_sliders;

        comma::name_value::parser csv_parser( "filename", ';', '=', false );
        auto csv = csv_parser.get< comma::csv::options >( unnamed[0] );
        std::string sliders_buffer;
        std::string sliders_binary;
        std::string comma;
        int i=0;
        if (csv.filename == "-") { 
            i=1; 
            if (unnamed.size() == 1) { COMMA_THROW( comma::exception, "You must specify at least one slider" ); }
        }
        std::vector< std::string > sliders_values( unnamed.size() ); // todo: fill from sliders so that sliders_buffer has default values        
        sliders_values={};
        snark::sliders::sliders_ptr sliders;
        sliders.reserve(unnamed.size());
        int offset = 0;
        for( ; i < unnamed.size(); i++ ){

            // Set up the buffer & check for binary or ascii
            auto s = csv_parser.get< comma::csv::options >( unnamed[i] );
            COMMA_ASSERT( s.binary() == csv.binary(), "expected all streams " << ( csv.binary() ? "binary" : "ascii" ) << "; got: '" << unnamed[0] << "' and '" << unnamed[i] << "'" );
            if( s.binary() ) { sliders_binary += comma + s.format().string(); comma = ","; }

            auto format_detail = comma::name_value::parser( "name", ';', '=', false ).get< snark::sliders::config< float > >( unnamed[i] );
            draw_slider<float>(mainLayout, format_detail, gui_sliders);
            auto slider = std::make_shared<snark::sliders::slider<float>>(offset, sizeof(float));
            offset += sizeof(float);
            slider->set(std::min(std::max(format_detail.default_value,format_detail.min),format_detail.max)).set_min(format_detail.min).set_max(format_detail.max);
            slider->set_name(format_detail.name);
            sliders_values.push_back( slider->as_string() );
            // sliders.at(i) = std::move(slider);
            sliders.push_back( std::move(slider) );
        }

    //             if(format_detail.format == "f"){
    //                 // auto s = new snark::sliders::slider<float>(0,0);
    //                 auto s = std::make_shared<snark::sliders::slider<float>>(0,0);
    //                 // auto s = reinterpret_cast<const snark::sliders::slider<float>*>(&slider);
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
            comma::csv::input_stream< snark::sliders::input > istream( std::cin, csv );
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


                    const snark::sliders::input* p = istream.read();
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
                    if( csv.flush ) { std::cout.flush(); }


                // Update the GUI
                } else if (ret == 0) {
                    app.processEvents();
                    sliders_values={};
                    for( unsigned int i = 0; i < sliders.size(); ++i ) { 
                        sliders_values.push_back(sliders[i]->as_string());
                    }
                    std::cout << comma::join( sliders_values, global_csv.delimiter ) << std::endl;

                } else {
                    perror("select");
                    return 1;
                }
            }
        } else
        {
            // Handle case with no stdin - i.e. we just publish the slider values. 
            float frequency = opts.value< float >( "--frequency,-f", 1 );
            while(1){
                for( unsigned int i = 0; i < sliders.size(); i++ ) { 
                    if (sliders[i]->type() == snark::sliders::slider_type::float_){
                        auto s = dynamic_cast<snark::sliders::slider<float>*>(sliders[i].get());
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



