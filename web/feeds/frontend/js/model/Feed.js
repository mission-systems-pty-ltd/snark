// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2011 The University of Sydney
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. Neither the name of the University of Sydney nor the
//    names of its contributors may be used to endorse or promote products
//    derived from this software without specific prior written permission.
//
// NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
// GRANTED BY THIS LICENSE.  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
// HOLDERS AND CONTRIBUTORS \"AS IS\" AND ANY EXPRESS OR IMPLIED
// WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
// BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
// OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
// IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// @authors Vinny Do, Vrushali Satpute
 
define('Feed', ["jquery", "jquery_timeago", "utils"], function ($)
{
    var Feed = function( feed_name, feed_path, config )
    {
        this.feed_name = feed_name;
        this.feed_path = feed_path;
        this.config = config;
        this.id = '#' + feed_path_to_id(this.feed_name);
        this.el = $(this.id);
        this.compact_icon = $(this.id + ' .panel-compact span');
        this.body = $(this.id + ' .panel-body');
        this.panel_refresh = $(this.id + ' .panel-refresh');
        this.status = $(this.id + ' .status');
        this.timestring = $(this.id + ' .timestring').hide();
        this.timeago = $(this.id + ' .timeago').timeago();
        this.target = $(this.id + ' .target');
        this.input_container = $(this.id + ' .inputs-group');
        this.hint = this.config.hint != undefined ? this.config.hint : "";
        this.height = this.config.height == undefined ? 269 : this.config.height;
        this.width = this.config.width == undefined ? 400 : this.config.width;
        this.interval = null;
        this.refresh_time = null;
        this.show = true;
        this.isMobile = false;
        this.fields = [];
        this.form = $( '<form>', { onSubmit: "return false" } );
        if( this.form_buttons_show == undefined ) { this.form_buttons_show = false; }
        if( this.form_buttons_names == undefined ) { this.form_buttons_names = []; }
        if( config.form != undefined )
        {
            this.extract_fields( config.form );
            var input_fields_link = this.feed_name + "-fields";
            var content_html;
            if( this.form_buttons_show )
            {
                content_html = '<div class="inputs-group form" id="' + input_fields_link + '"></div>';
            }
            else
            {
                content_html = '<a data-toggle="collapse" class="text-center" href="#' + input_fields_link + '">Input fields</a>'
                             + '<div class="inputs-group collapse" id="' + input_fields_link + '"></div>';
            }
            $( content_html ).insertBefore( this.target );
            this.input_container = $( this.id + ' .inputs-group' );
            this.ok_label = this.config.ok_label != undefined ? this.config.ok_label : "Submit"; //this.ok_label = this.config.ok_label != undefined ? this.config.ok_label : "Submit";
        }
        if( this.is_add_form() )
        {
            this.init_form();
            this.addListeners();
        }
    };
    Feed.prototype.reset = function ()
    {
        if( this.config.refresh.auto && !this.config.refresh.allow_input )
        {
            $(this.input_container).find("input").attr("readonly", "readonly");
            $(this.input_container).find("input").attr("title", "readonly");
            this.refresh();
        }
        else
        {
            this.clear_interval();
            $(this.input_container).find("input").removeAttr("readonly");
            $(this.input_container).find("input").removeAttr("title");
        }
    };
    Feed.prototype.extract_fields = function( form_elements )
    {
        if( form_elements == undefined ) { return; }
        let dropdowns = [];
        if( form_elements['dropdowns'] != undefined ) { dropdowns = form_elements['dropdowns']; } // quick and dirty
        if( form_elements['fields'] != undefined )
        {
            this.populate_path_values( form_elements['fields'], "", dropdowns );
            // this.buttons = form_elements['buttons'];
        }
        if( form_elements['buttons'] != undefined )
        {
            var buttons_config = form_elements['buttons'];
            var names = buttons_config['names'] == undefined ? [] : buttons_config['names'];
            var show = buttons_config['show'] == undefined ? false : buttons_config['show'];
            this.form_buttons_show = false;
            if( Array.isArray( names ) && names.length > 0 ) { this.form_buttons_show = true; this.form_buttons_names = names; }
            else if( Array.isArray( show ) && show.length > 0 ) { this.form_buttons_show = true; this.form_buttons_names = show; } // backward compatibility
            else if( show == "true" || ( typeof show == "boolean" && show ) ) { this.form_buttons_show = true; } // configs allow text or boolean i.e. "true"/true
            // this.buttons = form_elements['buttons'];
        }
        // else {
        //     this.populate_path_values(form_elements, "");
        // }
    };
    Feed.prototype.populate_path_values = function( form_elements, prefix, dropdowns )
    {
        for( var element in form_elements )
        {
            var value = form_elements[element];
            var type = typeof value;
            var p = get_prefix( element, prefix );
            if( type == "object" && !dropdowns.includes( p ) ) { this.populate_path_values( value, p, dropdowns ); } // Flatten 'object' field unless it is configured as a dropdown.
            else { this.fields[p] = value; }
            // console.log(element + " typeof " + (typeof element) + "   type= " + type);
        }
    };
    Feed.prototype.add_form = function ()
    {
        // if( this.form_buttons_show ) { this.input_container.append( this.form ); } // hm...
        // else { this.input_container.append( this.form ); }
        this.input_container.append( this.form );
    };
    Feed.prototype.init_form = function ()
    {
        this.input_container.empty();
        this.el.removeClass('panel-disabled').addClass('panel-enabled');
        // this.target.empty();
        this.form.empty();
        this.load_inputs( this.form );
        var this_ = this;
        // var combo = $('<select>');
        //
        // $("form").submit(function (event) {
        //     alert("Handler for .submit() called.");
        //     event.preventDefault();
        // });
        if( this.form_buttons_show || this.form_buttons_names != undefined && this.form_buttons_names.length > 0 )
        {
            var buttons_div = $('<div>', {class: "col-sm-12"});
            this.add_buttons( buttons_div );
            if( this.button_enabled( "clear" ) )
            {
                var clear = $( '<button/>',
                    {
                        text: 'Clear',
                        name: 'clear',
                        type: 'button',
                        class: "btn btn-default col-sm-3",
                        click: function()
                        {
                            $($(this).closest("form").find("input[type=text]")).each(function () { $(this).val(''); }); // clear text fields
                            $($(this).closest("form").find("input[type=checkbox]")).each(function () { $(this).prop( 'checked', false ); }); // clear checkboxes
                            $($(this).closest("form").find("select")).each(function () { $(this).val($(this).children()[0].text) }); // set dropdowns to the first option
                            $($(this).closest("form").find("button")).each(function () { $(this).attr('disabled', "disabled"); });
                            // $($(this).closest("form").find("button")).each(function () {
                            //     $(this).attr('disabled', "disabled");
                            // });
                        }
                    } );
                buttons_div.append( clear );
            }
            this.form.append( buttons_div );
        }
        // var num = this.buttons.length;
        // form.append(combo).append("<br>");
        // form.append(input).append("<br>");
        $(this.form).append($('<div>', {class: "clear"}));
        this.add_form();
        if( this.config.type == "start_stop" )
        {
            this.target.append( this.form );
        }
        else
        {
            //if( this.form_buttons_show ) { this.input_container.append( this.form ); } else { this.input_container.append( this.form ); }
            this.input_container.append( this.form );
        }
        var size = Object.keys( this.fields ).length;
        if( size > 0 ) { this.target.width( this_.width ); }
        else { $( this.target ).css( "min-width", function () { return 200; } ); }
    };
    Feed.prototype.button_enabled = function( name ) { return Array.isArray( this.form_buttons_names ) && this.form_buttons_names.length > 0 ? this.form_buttons_names.includes( name ) : this.form_buttons_show; }
    Feed.prototype.add_buttons = function( container )
    {
        this.refresh_time = new Date();
        var this_ = this;
        if( this.form_buttons_names != undefined && this.form_buttons_names.length > 0 ) // todo: add (reasonably) arbitrary list of buttons in the loop
        {
            for( var i in this.form_buttons_names )
            {
                var button_name = this.form_buttons_names[i]
                if( button_name == "clear" ) { continue; } // todo: quick and dirty, generalise
                var button = $('<button/>',
                    {
                        value: button_name,
                        text: button_name.charAt(0).toUpperCase() + button_name.slice(1),
                        class: "btn btn-primary col-sm-3", // todo: adjust button size based on number of buttons
                        click: function( event )
                        {
                            event.preventDefault();
                            var url = this_.get_url( encodeURIComponent( 'web/feeds/command' ) + '=' + encodeURIComponent( $(this).attr("value") ) );
                            if( url != undefined )
                            {
                                $.ajax(
                                {
                                    type: "GET",
                                    crossDomain: true,
                                    context: this,
                                    data: $( this ).closest( "form" ).serialize(),
                                    url: url
                                    // error: function (request, status, error) {
                                    // }
                                }).done( function ( data, textStatus, jqXHR )
                                {
                                    // var json = $.parseJSON(data);
                                    // Feed.prototype.onload_(data);
                                    // data = JSON.parse('{"output" : {"message": "Done. success", "x": { "a": 0, "b": 1 } },"status" :{"code": 0 , "message": "Success. Added successfully."}}');
                                    if( $( this ).attr( 'value' ) == 'query' ) // // todo? accept json data? currently, path-value only; quick and dirty for now
                                    {
                                        var p = data.split( '\n' );
                                        for( var i in p )
                                        {
                                            var k = p[i].split( '=' )[0];
                                            if( k.trim().length == 0 ) { continue; }
                                            var input = $( this.form ).find( 'input[name="' + k + '"]' );
                                            if( input != undefined ) { input.val( p[i].slice( k.length + 1 ) ); }
                                        }
                                    }
                                    this_.onload( data );
                                }).fail( function ( jqXHR, textStatus, errorThrown )
                                {
                                    // console.log(jqXHR);
                                    if( jqXHR.readyState == 0 ) { errorThrown = "Connection Refused" }
                                    this_.update_error( this_, errorThrown );
                                });
                            }
                        }
                    });
                container.append( button );
                container.append($('<label/>', {class: "col-sm-1"})); // todo: adjust padding size based on number of buttons
            }
            //container.append($('<label/>', {class: "col-sm-1"})); // todo: adjust padding size based on number of buttons
        }
        else
        {
            if( this.button_enabled( "ok" ) )
            {
                var submit = $('<button/>',
                    {
                        value: "submit",
                        text: this.ok_label,
                        class: "btn btn-primary col-sm-3",
                        click: function( event )
                        {
                            event.preventDefault();
                            var url = this_.get_url();
                            if( url != undefined )
                            {
                                $.ajax({
                                    type: "GET",
                                    crossDomain: true,
                                    context: this,
                                    data: $(this).closest('form').serialize(),
                                    url: url
                                    // error: function (request, status, error) {
                                    // }
                                }).done(function ( data, textStatus, jqXHR ) {
                                    // var json = $.parseJSON(data);
                                    // Feed.prototype.onload_(data);
                                    // data = JSON.parse('{"output" : {"message": "Done. success", "x": { "a": 0, "b": 1 } },"status" :{"code": 0 , "message": "Success. Added successfully."}}');
                                    this_.onload( data );
                                }).fail( function ( jqXHR, textStatus, errorThrown ) {
                                    // console.log(jqXHR);
                                    if( jqXHR.readyState == 0 ) { errorThrown = "Connection Refused" }
                                    this_.update_error( this_, errorThrown );
                                });
                            }
                        }
                    });
                container.append( submit );
                container.append($('<label/>', {class: "col-sm-1"}));
            }
            else
            {
                container.append($('<label/>', {class: "col-sm-4"}));
            }
            container.append($('<label/>', {class: "col-sm-5"}));
        }
    };
    Feed.prototype.addListeners = function()
    {
        $($(this.form).find("input")).on( "input", function() // re-enable submit button whenever an input element changes
        {
            $($(this).closest("form").find("button")).each( function() { $(this).removeAttr( 'disabled' ); } );
        } );
        $($(this.form).find("select")).on( "change", function() // re-enable submit button whenever a select element changes
        {
            $($(this).closest("form").find("button")).each( function() { $(this).removeAttr( 'disabled' ); } );
        } );
    };
    var get_prefix = function( key, prefix ) { return prefix.length == 0 ? key : ( isNaN( key ) ? prefix + "/" + key : prefix + "[" + key + "]" ); };
    Feed.prototype.set_interval = function()
    {
        this.interval = setInterval(this.preload.bind(this), this.config.refresh.interval * 1000);
        this.status.removeClass('text-muted glyphicon-stop').addClass('text-success glyphicon-refresh');
        this.el.removeClass('panel-disabled').addClass('panel-enabled');
        var gui_folder = $(gui.__folders[this.feed_path].__ul);
        if( globals.isMobile )
        {
            gui_folder.closest('li').find('a.ui-collapsible-heading-toggle').first().removeClass('panel-disabled').addClass('panel-enabled');
        }
        else
        {
            gui_folder.find('.title').first().removeClass('panel-disabled').addClass('panel-enabled');
        }
    };
    Feed.prototype.clear_interval = function ()
    {
        clearInterval(this.interval);
        delete pending[this.feed_name];
        if( !( this.target.hasClass( 'form' ) || this.input_container.hasClass( 'form' ) ) )
        {
            this.status.removeClass('text-success glyphicon-refresh').addClass('text-muted glyphicon-stop');
            this.el.removeClass('panel-enabled').addClass('panel-disabled');
            var gui_folder = $(gui.__folders[this.feed_path].__ul);
            if( globals.isMobile )
            {
                gui_folder.closest('li').find('a.ui-collapsible-heading-toggle').removeClass('panel-enabled').addClass('panel-disabled');
            }
            else
            {
                gui_folder.find('.title').removeClass('panel-enabled').addClass('panel-disabled');
            }
        }
        else
        {
            // this.el.removeClass('panel-enabled').addClass('panel-disabled');
        }
    };
    Feed.prototype.refresh = function ()
    {
        if( !this.form_buttons_show || this.config.refresh.auto )
        {
            this.clear_interval();
            this.preload();
            if( this.config.refresh.auto ) { this.set_interval(); }
        }
        if( this.hint != "" )
        {
            $(this.target).attr("rel", "tooltip");
            $(this.target).attr("data-placement", "top");
            $(this.target).attr("data-html", "true");
            $(this.target).attr("data-toggle", "tooltip");
            $(this.target).attr("data-original-title", this.hint);
            $(this.target).tooltip();
        }
    };
    Feed.prototype.update_view = function ()
    {
        if( this.config.view == 'show' )
        {
            this.el.show();
            this.body.show();
            this.compact_icon.removeClass('glyphicon-resize-full').addClass('glyphicon-resize-small');
        }
        else if( this.config.view == 'compact' )
        {
            this.el.show();
            this.body.hide();
            this.compact_icon.removeClass('glyphicon-resize-small').addClass('glyphicon-resize-full');
        }
        else if( this.config.view == 'hide' )
        {
            this.el.hide();
        }
    };
    Feed.prototype.load_inputs = function( container )
    {
        var is_disabled = false;
        if( this.config.refresh.auto && !this.config.refresh.allow_input ) { is_disabled = true; }
        for( var field in this.fields )
        {
            var row = $( '<div>', { class: "form" } );
            var label = $('<label>',
                {
                    text: field,
                    class: "col-sm-4 text-nowrap"
                });
            var each = $( '<div>', { class: " col-sm-8" } );
            var input;
            var field_value = this.fields[field];
            if( Array.isArray( field_value ) )
            {
                // Array values become a dropdown selector. These fields must have been
                // configured using the `"dropdown": ["field-name"]` syntax, or otherwise
                // they'd already have been flattened to individual fields by
                // `populate_path_values`.
                input = $('<select>',
                {
                    class: "form-control ",
                    name: field,
                    value: field_value[0]
                });
                field_value.forEach( f => { input.append($('<option>').append(f)); } );
            }
            else
            {
                var input_type = typeof field_value == 'number' ? 'number' : typeof field_value == 'boolean' ? 'checkbox' : 'text';
                if( input_type == 'checkbox' )
                {
                    input = $('<input>',
                    {
                        type: 'checkbox',
                        class: "form-control ",
                        name: field,
                        checked: field_value,
                        value: true
                    });
                }
                else
                {
                    input = $('<input>',
                    {
                        type: input_type,
                        class: "form-control ",
                        name: field,
                        value: field_value
                    });
                }
            }
            if( is_disabled )
            {
                $(input).prop('readonly', "readonly");
                $(input).prop('title', "readonly");
            }
            each.append(input);
            row.append(label).append(each);
            container.append(row).append($('<div>', {class: "clear"})); // this.target.append(row);
        }
    };
    Feed.prototype.preload = function()
    {
        if( pending[this.feed_name] ) { return; }
        pending[this.feed_name] = true;
        this.refresh_time = new Date();
        this.status.fadeTo(1000, 0.4);
        this.load();
    };
    Feed.prototype.is_add_form = function()
    {
        var size = Object.keys(this.fields).length;
        return ( size > 0 && this.config.type != "form" && this.config.type != 'stream' ) || this.config.type == "start_stop";
    };
    Feed.prototype.is_feed_inputs = function()
    {
        var size = Object.keys( this.fields ).length;
        return size > 0 && this.config.type != "form" && this.config.type != "start_stop" && this.config.type != 'stream';
    };
    Feed.prototype.update_time = function()
    {
        var timestring = this.refresh_time.toString();
        //alert(timestring);
        var timezone = timestring.match(/\((.*)\)$/);
        if( timezone != undefined && timezone.length == 2 && timezone[1].match(/ /) )
        {
            var short_timezone = timezone[1].split(' ').map( function( word, index ) { return word[0]; }).join( '' );
            timestring = timestring.replace(/\(.*\)$/, '(' + short_timezone + ')');
        }
        this.timestring.text(timestring);
        this.timeago.attr('datetime', this.refresh_time.toISOString()).timeago('updateFromDOM');
    };
    Feed.prototype.onload = function( data )
    {
        if( this.form_buttons_show ) { this.removeOldStatus(); }
        this.update_time();
        this.status.finish().fadeTo( 0, 1 );
        if( this.config.alert ) { this.alert(!data || !data.length); }
        this.onload_( data );
        delete pending[this.feed_name];
    };
    Feed.prototype.onerror = function()
    {
        this.update_time();
        this.status.finish().fadeTo( 0, 1 );
        this.onload_();
        delete pending[this.feed_name];
        if( this.config.alert ) { this.alert( true ); }
    };
    Feed.prototype.removeOldStatus = function ()
    {
        $( this.form ).find( '.error-message' ).remove();
        this.el.removeClass( 'panel-disabled' );
    };
    Feed.prototype.update_error = function( this_, error )
    {
        this_.update_time();
        if( error.trim().length == 0 ) { error = "Please check configuration!"; } // todo: better error message
        this.status.finish().fadeTo(0, 1);
        this.onload_();
        $( this.form ).find( ".error-message" ).remove();
        this.el.addClass( 'panel-disabled' );
        $( this.form ).append( $( '<label>',
        {
            class: "error-message col-sm-11 ",
            text: ( status.message ? status.message : "Error : " + error )
        }));
        $( this.form ).append( $( '<div>', { class: "clear" } ) );
    };
    Feed.prototype.get_url = function( params = undefined )
    {
        var url = this.config.url;
        var timestamp = new Date().getTime();
        url += ( url.indexOf( '?' ) < 0 ? '?_=' : '&_=' ) + timestamp;
        if( params != undefined ) { url += ( url.indexOf( '?' ) < 0 ? ' ' : '&' ) + params; }
        if( this.is_feed_inputs() )
        {
            var form_params = $( this.input_container ).find( "form" ).serialize();
            if( form_params.length > 0 ) { url += ( url.indexOf( '?' ) < 0 ? ' ' : '&' ) + form_params; }
        }
        return url;
    };
    Feed.prototype.alert = function (on)
    {
        var gui_element = gui.__folders[this.feed_path];
        if( gui_element != undefined )
        {
            var gui_folder = $( gui_element.__ul );
            if( on )
            {
                this.el.addClass('panel-alert');
                if (globals.isMobile) { gui_folder.closest('li').find('a.ui-collapsible-heading-toggle').first().addClass('panel-alert'); }
                else { gui_folder.find('.title').first().addClass('panel-alert'); }
                if( globals.alert_beep ) { globals.beep(); }
            }
            else
            {
                if( globals.isMobile ) { gui_folder.closest('li').find('a.ui-collapsible-heading-toggle').removeClass('panel-alert'); }
                else { gui_folder.find('.title').removeClass('panel-alert'); }
                this.el.removeClass('panel-alert');
            }
        }
    };
    Feed.views = ['show', 'compact', 'hide'];
    return Feed;
});
