// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2016 The University of Sydney

#ifndef SNARK_SENSORS_VIMBA_ATTRIBUTE_H_
#define SNARK_SENSORS_VIMBA_ATTRIBUTE_H_

#include <string>
#include <vector>
#include <VimbaCPP/Include/Feature.h>

namespace snark { namespace vimba {

class attribute
{
    public:
        attribute( AVT::VmbAPI::FeaturePtr feature );
        attribute( const attribute& attribute );

        const std::string& name() const { return name_; }
        VmbFeatureDataType type() const { return type_; }
        const std::string& description() const { return description_; }
        const std::vector< std::string >& allowed_values() const { return allowed_values_; }

        unsigned int int_value() const { return value_.int_value; }

        const std::string& value_as_string() const { return value_as_string_; }
        const char*        type_as_string() const;
        std::string        allowed_values_as_string() const;

        void set( const std::string& value );

    private:
        void init_value();
        void init_allowed_values();

        AVT::VmbAPI::FeaturePtr feature_;
        std::string name_;
        VmbFeatureDataType type_;

        // Non-string types
        union value
        {
            VmbInt64_t enum_value;
            VmbBool_t  bool_value;
            double     float_value;
            VmbInt64_t int_value;
        } value_;

        std::string value_as_string_;      // value as string
        std::string description_;
        std::vector< std::string > allowed_values_;
};

} } // namespace snark { namespace vimba {

#endif // SNARK_SENSORS_VIMBA_ATTRIBUTE_H_
