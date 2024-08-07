// Copyright (c) 2011 The University of Sydney

/// @author Vsevolod Vlaskine

#include <fstream>
#include <iostream>
#include "qglobal.h"
#if QT_VERSION >= 0x050000
#include <QtWidgets>
#else
#include <QtGui>
#endif
#include <QFileDialog>
#include <QFrame>
#include <QLabel>
#include <QLayout>
#include <comma/base/exception.h>
#include "action.h"
#include "main_window.h"

namespace snark { namespace graphics { namespace view {

MainWindow::MainWindow( const std::string& title
                      , const std::shared_ptr< snark::graphics::view::controller >& c
                      , const std::vector< int >& window_geometry
                      , bool minimalistic
                      , bool show_fields )
    : controller( c )
    , _file_frame_visible( !minimalistic && controller->readers.size() > 1 )
    , _show_fields( show_fields )
    , _escape( QKeySequence( Qt::Key_Escape ), this, SLOT( close() ) )
    , _ctrl_w( QKeySequence( tr( "Ctrl+W" ) ), this, SLOT( close() ) )
{
    QMenu* file_menu = menuBar()->addMenu( "File" );
    menuBar()->addMenu( file_menu );
    file_menu->addAction( new Action( "Load Camera Config...", boost::bind( &MainWindow::load_camera_config, this ) ) );
    file_menu->addAction( new Action( "Save Camera Config...", boost::bind( &MainWindow::save_camera_config, this ) ) );
    file_menu->addAction( new Action( "Print Window Geometry", boost::bind( &MainWindow::_print_window_geometry, this ), "Ctrl+G" ) );

    m_fileFrame = new QFrame;
    m_fileFrame->setFrameStyle( QFrame::Plain | QFrame::NoFrame );
    m_fileFrame->setFixedWidth( 300 );
    m_fileFrame->setContentsMargins( 0, 0, 0, 0 );
    m_fileLayout = new QGridLayout;
    m_fileLayout->setSpacing( 0 );
    m_fileLayout->setContentsMargins( 5, 5, 5, 0 );

    QLabel* filenameLabel = new QLabel( "<b>file/stream</b>" );
    QLabel* visibleLabel = new QLabel( "<b>visible</b>" );
    m_fileLayout->addWidget( filenameLabel, 0, 0, Qt::AlignLeft | Qt::AlignTop );
    m_fileLayout->addWidget( visibleLabel, 0, 1, Qt::AlignRight | Qt::AlignTop );
    m_fileLayout->setRowStretch( 0, 0 );
    m_fileLayout->setColumnStretch( 0, 10 ); // quick and dirty
    m_fileLayout->setColumnStretch( 1, 1 );
    m_fileFrame->setLayout( m_fileLayout );

    QFrame* frame = new QFrame;
    frame->setFrameStyle( QFrame::Plain | QFrame::NoFrame );
    frame->setContentsMargins( 0, 0, 0, 0 );
    QGridLayout* layout = new QGridLayout;
    layout->setContentsMargins( 0, 0, 0, 0 );
    layout->setSpacing( 0 );

    //QScrollArea* area = new QScrollArea( this );
    //area->setWidget( m_fileFrame );
    //area->show();

    layout->addWidget( m_fileFrame, 0, 0 );
    viewer_t* viewer = controller_traits< snark::graphics::view::controller >::get_widget( controller );
    if( window_geometry[2] > 0 || window_geometry[3] > 0 ) { viewer->resize( 100, 100 ); } // quick and dirty, to make it possible to have small main window
    #if QT_VERSION >= 0x050000
        #if Qt3D_VERSION==1
        layout->addWidget( QWidget::createWindowContainer( viewer ), 0, 1 );
        #elif Qt3D_VERSION>=2
        layout->addWidget( viewer, 0, 1 );
        #endif
    #else
        layout->addWidget( viewer, 0, 1 );
    #endif
    layout->setColumnStretch( 0, 0 );
    layout->setColumnStretch( 1, 1 );
    layout->setRowMinimumHeight( 0, viewer->size().height() );
    layout->setColumnMinimumWidth( 1, viewer->size().width() );
    frame->setLayout( layout );
    //resize( viewer->size().width(), viewer->size().height() ); // resize( 640, 480 );

    QFrame* outer_frame = new QFrame; // lame, todo: use layout config files
    outer_frame->setFrameStyle( QFrame::Plain | QFrame::NoFrame );
    outer_frame->setContentsMargins( 0, 0, 0, 0 );
    QGridLayout* outer_layout = new QGridLayout;
    outer_layout->setContentsMargins( 0, 0, 0, 0 );
    outer_layout->setSpacing( 0 );
    outer_layout->setColumnStretch( 0, 0 );
    outer_layout->setColumnStretch( 1, 0 );
    outer_layout->setRowMinimumHeight( 0, viewer->size().height() );
    outer_layout->setColumnMinimumWidth( 0, viewer->size().width() );
    outer_layout->addWidget( frame, 0, 0 );
    outer_frame->setLayout( outer_layout );

    setCentralWidget( outer_frame ); // setCentralWidget( frame );
    _view_menu = menuBar()->addMenu( "View" );
    ToggleAction* toggle_action = new ToggleAction( "File Panel", boost::bind( &MainWindow::toggle_file_frame, this, boost::placeholders::_1 ) );
    toggle_action->setChecked( _file_frame_visible );
    _view_menu->addAction( toggle_action );

    updateFileFrame();
    toggle_file_frame( _file_frame_visible );
    setWindowTitle( &title[0] );

    #if Qt3D_VERSION>=2
    auto modeMenu = menuBar()->addMenu( "Modes" );
    toggle_action = new ToggleAction( "Block Mode", boost::bind( &snark::graphics::view::viewer_t::toggle_block_mode, viewer, boost::placeholders::_1 ) );
    toggle_action->setShortcut( QKeySequence( "Ctrl+B" ) );
    toggle_action->setChecked( viewer->click_mode.double_right_click.mode() == click_mode::double_right_click_t::modes::block );
    modeMenu->addAction( toggle_action );
    toggle_action = new ToggleAction( "Label Mode", boost::bind( &snark::graphics::view::viewer_t::toggle_label_mode, viewer, boost::placeholders::_1 ) );
    toggle_action->setShortcut( QKeySequence( "Ctrl+L" ) );
    toggle_action->setChecked( viewer->click_mode.double_right_click.mode() == click_mode::double_right_click_t::modes::label );
    modeMenu->addAction( toggle_action );
    #endif

    viewer->setFocus();
    setGeometry( window_geometry[0], window_geometry[1], window_geometry[2], window_geometry[3] );
    viewer->resize( -1, -1 ); //viewer->resize( window_geometry[2], window_geometry[3] );
    //std::cerr << "==> main window: viewer size: " << viewer->size().width() << "," << viewer->size().height() << std::endl;
    viewer->update_projection(); // quick and dirty
}

CheckBox::CheckBox( boost::function< void( bool ) > f ) : m_f( f ) { connect( this, SIGNAL( toggled( bool ) ), this, SLOT( action( bool ) ) ); }

void CheckBox::action( bool checked ) { m_f( checked ); }

void MainWindow::showFileGroup( std::string const& name, bool shown )
{
    FileGroupMap::iterator it = m_userGroups.find( name );
    FileGroupMap::iterator end = m_userGroups.end();
    if( it == m_userGroups.end() )
    {
        it = m_fieldsGroups.find( name );
        end = m_fieldsGroups.end();
    }
    if( it == end ) { std::cerr << "view-points: warning: file group \"" << name << "\" not found" << std::endl; }
    for( std::size_t i = 0; i < it->second.size(); ++i ) { it->second[i]->setCheckState( shown ? Qt::Checked : Qt::Unchecked ); }
}

void MainWindow::updateFileFrame() // quick and dirty
{
    for( std::size_t i = 0; i < controller->readers.size(); ++i ) // quick and dirty: clean
    {
        for( unsigned int k = 0; k < 2; ++k )
        {
            if( m_fileLayout->itemAtPosition( i + 1, k ) == NULL ) { continue; }
            QWidget* widget = m_fileLayout->itemAtPosition( i + 1, k )->widget();
            m_fileLayout->removeWidget( widget );
            delete widget;
        }
    }
    bool same_fields = true;
    std::string fields;
    for( std::size_t i = 0; same_fields && i < controller->readers.size(); ++i )
    {
        if( i == 0 ) { fields = controller->readers[0]->options.fields; } // quick and dirty
        else { same_fields = controller->readers[i]->options.fields == fields; }
    }
    m_userGroups.clear();
    m_fieldsGroups.clear();
    for( std::size_t i = 0; i < controller->readers.size(); ++i )
    {
        static const std::size_t maxLength = 30; // arbitrary
        std::string title = controller->readers[i]->title;
        if( title.length() > maxLength )
        {
            #ifdef WIN32
            std::string leaf = comma::split( title, '\\' ).back();
            #else
            std::string leaf = comma::split( title, '/' ).back();
            #endif
            title = leaf.length() >= maxLength ? leaf : std::string( "..." ) + title.substr( title.length() - maxLength );
        }
        if( _show_fields && !same_fields ) { title += ": \"" + controller->readers[i]->options.fields + "\""; }
        m_fileLayout->addWidget( new QLabel( &title[0] ), i + 1, 0, Qt::AlignLeft | Qt::AlignTop );
        CheckBox* view_box = new CheckBox( boost::bind( &Reader::show, boost::ref( *controller->readers[i] ), boost::placeholders::_1 ) );
        view_box->setCheckState( controller->readers[i]->show() ? Qt::Checked : Qt::Unchecked );
        connect( view_box, SIGNAL( toggled( bool ) ), this, SLOT( update_view() ) ); // redraw when box is toggled
        view_box->setToolTip( ( std::string( "check to make " ) + title + " visible" ).c_str() );
        m_fileLayout->addWidget( view_box, i + 1, 1, Qt::AlignRight | Qt::AlignTop );
        m_fileLayout->setRowStretch( i + 1, i + 1 == controller->readers.size() ? 1 : 0 );
        m_fieldsGroups[ controller->readers[i]->options.fields ].push_back( view_box );
        if ( !controller->readers[i]->groups.empty() )
        {
            auto group_list = comma::split( controller->readers[i]->groups, ',' );
            for( auto& gi : group_list ) { m_userGroups[ gi ].push_back( view_box ); }
        }
    }
    std::size_t i = 1 + controller->readers.size();
    // for( std::size_t j = 0; j < controller->readers.size(); ++j ) // quick and dirty, just for now... todo! certainly make collapsable reader widget with properties including colorbar
    // {
    //     // todo: something like: https://code.qt.io/cgit/qt/qtdatavis3d.git/tree/examples/datavisualization/surface/main.cpp?h=5.15
    //     const auto& c = controller->readers[j].colored();
    //     if( !c.extents() ) { continue; }
    //     auto e = *c.extents();        
    //     QLinearGradient g(0, 0, 1, 100);
    //     grBtoY.setColorAt(1.0, c.color( Eigen::Vector3d::Zero(), 0, e.first, color_t() ) );
    //     grBtoY.setColorAt(0.67, ...);
    //     grBtoY.setColorAt(0.33, ...);
    //     grBtoY.setColorAt(0.0, c.color( Eigen::Vector3d::Zero(), 0, e.second, color_t() ) );
    //     QPixmap pm(24, 100);
    //     QPainter pmp(&pm);
    //     pmp.setBrush(QBrush(grBtoY));
    //     pmp.setPen(Qt::NoPen);
    //     pmp.drawRect(0, 0, 24, 100);
    //     QPushButton *gradientBtoYPB = new QPushButton(widget);
    //     gradientBtoYPB->setIcon(QIcon(pm));
    //     gradientBtoYPB->setIconSize(QSize(24, 100));
    //     ++i;
    // }
    m_fileLayout->addWidget( new QLabel( "<b>groups</b>" ), i++, 0, Qt::AlignLeft | Qt::AlignTop );
    for( FileGroupMap::const_iterator it = m_userGroups.begin(); it != m_userGroups.end(); ++it, ++i )
    {
        m_fileLayout->addWidget( new QLabel( ( "\"" + it->first + "\"" ).c_str() ), i, 0, Qt::AlignLeft | Qt::AlignTop );
        CheckBox* view_box = new CheckBox( boost::bind( &MainWindow::showFileGroup, this, it->first, boost::placeholders::_1 ) );
        //view_box->setCheckState( Qt::Checked );
        view_box->setToolTip( ( std::string( "check to make files within group \"" ) + it->first + "\" visible" ).c_str() );
        m_fileLayout->addWidget( view_box, i, 1, Qt::AlignRight | Qt::AlignTop );
        m_fileLayout->setRowStretch( i, i + 1 == controller->readers.size() ? 1 : 0 );
    }
    m_fileLayout->addWidget( new QLabel( "<b>groups by fields</b>" ), i++, 0, Qt::AlignLeft | Qt::AlignTop );
    for( FileGroupMap::const_iterator it = m_fieldsGroups.begin(); it != m_fieldsGroups.end(); ++it, ++i )
    {
        m_fileLayout->addWidget( new QLabel( ( "\"" + it->first + "\"" ).c_str() ), i, 0, Qt::AlignLeft | Qt::AlignTop );
        CheckBox* view_box = new CheckBox( boost::bind( &MainWindow::showFileGroup, this, it->first, boost::placeholders::_1 ) );
        //view_box->setCheckState( Qt::Checked );
        view_box->setToolTip( ( std::string( "check to make files with fields \"" ) + it->first + "\" visible" ).c_str() );
        m_fileLayout->addWidget( view_box, i, 1, Qt::AlignRight | Qt::AlignTop );
        m_fileLayout->setRowStretch( i, i + 1 == controller->readers.size() + fields.size() ? 1 : 0 );
    }
}

void MainWindow::update_view()
{
    controller->update_view();
    controller_traits< snark::graphics::view::controller >::get_widget( controller )->setFocus();

}

void MainWindow::toggle_file_frame( bool visible )
{
    _file_frame_visible = visible;
    if( visible ) { m_fileFrame->show(); } else { m_fileFrame->hide(); }
}

void MainWindow::closeEvent( QCloseEvent * ) { controller->shutdown(); }

void MainWindow::_print_window_geometry() const
{
    auto g = geometry();
    std::cerr << g.left() << "," << g.top() << "," << g.width() << "," << g.height() << std::endl;
}

void MainWindow::load_camera_config()
{
    QString filename = QFileDialog::getOpenFileName( this, "Load Camera Config" );
    if( filename.isNull() ) { return; }
    std::cerr << "view-points: loading camera config from '" << filename.toStdString() << "'" << std::endl;
    controller->load_camera_config( filename.toStdString() );
    std::cerr << "view-points: loaded camera config from '" << filename.toStdString() << "'" << std::endl;
}

void MainWindow::save_camera_config()
{
    QString filename = QFileDialog::getSaveFileName( this, "Save Camera Config" );
    if( filename.isNull() ) { return; }
    std::cerr << "view-points: saving camera config to '" << filename.toStdString() << "'" << std::endl;
    std::ofstream fs( filename.toStdString() );
    if( !fs.is_open() ) { COMMA_THROW( comma::exception, "failed to open '" << filename.toStdString() << "'" ); }
    controller->write_camera_config( fs );
    std::cerr << "view-points: saved camera config to '" << filename.toStdString() << "'" << std::endl;
}

} } } // namespace snark { namespace graphics { namespace view {
