// Copyright (c) 2017 The University of Sydney

/// @author Navid Pirmarzdashti

#pragma once
#include "label_shader.h"
#include "types.h"

namespace snark { namespace graphics { namespace qopengl {

class text_label : public label
{
    public:
        text_label( const Eigen::Vector3d& position, const std::string& text, color_t color, unsigned int font_size );
        virtual void update();

    protected:
        virtual void draw( QPainter& painter );

    private:
        Eigen::Vector3d position;
        std::string text;
        color_t color;
        int width;
        int height;
        unsigned int font_size;
};
    
} } } // namespace snark { namespace graphics { namespace qopengl {
    
