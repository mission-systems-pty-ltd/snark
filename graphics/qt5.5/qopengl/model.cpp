// Copyright (c) 2017 The University of Sydney

/// @author Navid Pirmarzdashti

#include <iostream>
#include <comma/base/exception.h>
#include "model.h"

namespace snark { namespace graphics { namespace qopengl {

model::model() : scene(NULL) { }
void model::import(const std::string& file_name)
{
//         aiSetImportPropertyInteger(props,AI_CONFIG_PP_SBP_REMOVE, aiPrimitiveType_LINE | aiPrimitiveType_POINT);
//         ReadFileEx(... prop)
    
//         aiProcess_PreTransformVertices | aiProcess_GenUVCoords | aiProcess_TransformUVCoords 
    scene = importer.ReadFile(file_name, aiProcess_Triangulate | aiProcess_GenNormals | aiProcess_JoinIdenticalVertices | aiProcess_SortByPType);
//             aiProcess_CalcTangentSpace       |
//             aiProcess_Triangulate            |
//             aiProcess_JoinIdenticalVertices  |
//             aiProcess_SortByPType);
    if(!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE) { COMMA_THROW( comma::exception, "failed to import model file: "<<file_name); }
//     debug();
}

void model::make_meshes( mesh_shader& shader )
{
    if(!scene || !scene->mMeshes) { COMMA_THROW( comma::exception, "scence is null!"); }
    //std::cerr<<"model::make_meshes "<<scene->mNumMeshes<<std::endl;
    node_make_meshes( scene->mRootNode,shader );
    //std::cerr<<"/model::make_meshes"<<std::endl;
}

void model::node_make_meshes(aiNode* node,mesh_shader& shader)
{
    if( !node ) { COMMA_THROW( comma::exception,"node is null!" ); }
    for( unsigned int i = 0; i < node->mNumMeshes; ++i )
    {
        aiMesh* mm = scene->mMeshes[ node->mMeshes[i] ];
        if( !mm ) { COMMA_THROW(comma::exception,"mesh is null!");}
        qopengl::mesh* mesh = new qopengl::mesh();
        shader.meshes.push_back( std::shared_ptr< qopengl::mesh >( mesh ) );
        if( !mm->mVertices ) { COMMA_THROW( comma::exception, "vertices is null!" ); }
        mesh->update( mm->mVertices, mm->mNumVertices );
    }
    for( unsigned int j = 0; j < node->mNumChildren; ++j ) { node_make_meshes( node->mChildren[j], shader ); }
}

void model::debug()
{
    std::cerr<<"view-points: import msh "<<scene->mNumMeshes<<", mt "<<scene->mNumMaterials<<", tx "<<scene->mNumTextures<<", cam "<<scene->mNumCameras<<std::endl;
    if(scene->HasMeshes())
    {
        aiMesh* mesh=scene->mMeshes[0];
        std::cerr << "first mesh v "<<mesh->mNumVertices<<", f "<<mesh->mNumFaces<<", mt "<<mesh->mMaterialIndex<<", uv[0] "<<mesh->mNumUVComponents[0]<<std::endl;
        std::cerr << "view-points: colors ";
        for(unsigned i=0;i<AI_MAX_NUMBER_OF_COLOR_SETS;i++) { std::cerr << mesh->mColors[i]<<", "; }
        std::cerr<<std::endl;
        if(scene->HasMaterials())
        {
            aiMaterial* mat=scene->mMaterials[mesh->mMaterialIndex];
            std::cerr<<"view-points: material "<<mat->mNumProperties<<std::endl;
            for(unsigned i=0;i<mat->mNumProperties;i++)
            {
                aiMaterialProperty* prop=mat->mProperties[i];
                std::cerr<<"view-points: prop "<<prop->mKey.C_Str()<<" "<<prop->mDataLength<<" ";
                if(prop->mType==aiPTI_String)
                {
                    aiString str;
                    mat->Get(prop->mKey.C_Str(),0,0,str);
                    std::cerr<<str.C_Str();
                }
                else
                {
                    for(unsigned j=0;j<prop->mDataLength/4;j++)
                    {
                        switch(prop->mType)
                        {
                            case aiPTI_Float:
                                if(!j) std::cerr<<"float ";
                                std::cerr<<((float*)prop->mData)[j]<<" ";
                                break;
                            case aiPTI_Integer:
                                if(!j) std::cerr<<"int ";
                                std::cerr<<((int*)prop->mData)[j]<<" ";
                                break;
                            default:
                                break;
                        }
                    }
                }
                std::cerr<<std::endl;
            }
        }
    }
}
    
} } } // namespace snark { namespace graphics { namespace qopengl {
    
