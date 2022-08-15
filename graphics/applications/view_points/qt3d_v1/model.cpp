// Copyright (c) 2017 The University of Sydney

#include <comma/base/exception.h>
#include <Qt3D/qglpainter.h>
#include <Qt3D/qglabstractscene.h>
#include <Qt3D/qglscenenode.h>
#include "model.h"

namespace snark { namespace graphics { namespace view { namespace qt3d_v1 {
    
model::model(): m_scene(NULL) {}

void model::load( const std::string& file ) { m_scene = QGLAbstractScene::loadScene( QLatin1String( file.c_str() ) ); }

void model::render( QGLPainter* painter )
{
    if( m_scene == NULL ) { COMMA_THROW( comma::exception, "scene is NULL"); }
    QGLSceneNode* node = m_scene->mainNode();
    //     painter->setStandardEffect( QGL::LitMaterial ); // no effect ?
    node->draw( painter );
}
    
} } } } // namespace snark { namespace graphics { namespace view { namespace qt3d_v1 {
    
