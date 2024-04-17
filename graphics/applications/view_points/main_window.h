// Copyright (c) 2011 The University of Sydney

/// @author Vsevolod Vlaskine

#pragma once

#include <QCheckBox>
#include <QMainWindow>
#include <QShortcut>
#include "controller.h"

QT_BEGIN_NAMESPACE
class QAction;
class QActionGroup;
class QFrame;
class QGridLayout;
class QMenu;
class QToolBar;
QT_END_NAMESPACE

namespace snark { namespace graphics { namespace view {

class CheckBox;

class MainWindow : public QMainWindow
{
    Q_OBJECT

    public:
        MainWindow( const std::string& title
                  , const std::shared_ptr< snark::graphics::view::controller >& controller
                  , const std::vector< int >& window_geometry
                  , bool minimalistic
                  , bool show_fields );
        void toggle_file_frame( bool shown );

    private slots:
        void update_view();

    private:
        QMenu* _view_menu;
        std::shared_ptr< snark::graphics::view::controller > controller;
        QFrame* m_fileFrame;
        QGridLayout* m_fileLayout;
        bool _file_frame_visible{true};
        bool _show_fields{true};
        typedef std::map< std::string, std::vector< CheckBox* > > FileGroupMap;
        FileGroupMap m_userGroups; // quick and dirty
        FileGroupMap m_fieldsGroups; // quick and dirty
        QShortcut _escape;
        QShortcut _ctrl_w;

        void closeEvent( QCloseEvent* event );
        void updateFileFrame();
        void makeFileGroups();
        void showFileGroup( std::string const& name, bool shown );
        void load_camera_config();
        void save_camera_config();
        void _print_window_geometry() const;
};

class CheckBox : public QCheckBox // quick and dirty
{
    Q_OBJECT

    public:
        CheckBox( boost::function< void( bool ) > f );

    public slots:
        void action( bool checked );

    private:
        boost::function< void( bool ) > m_f;
};

} } } // namespace snark { namespace graphics { namespace view {
