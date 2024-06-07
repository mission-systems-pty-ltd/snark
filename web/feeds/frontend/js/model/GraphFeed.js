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

/// @authors Vinny Do, Vrushali Satpute

define('GraphFeed', ["jquery", "Feed"], function ($)
{
    var Feed = require('Feed');
    var GraphFeed = function( feed_name, feed_path, config )
    {
        this.base = Feed;
        this.base( feed_name, feed_path, config );
        this.default_threshold = { value: this.config.graph.max, color: '#5cb85c' };
        this.default_exceeded_threshold = { value: Number.MAX_VALUE, color: '#d9534f', alert: true };
        this.text = $(this.id + ' .graph-text');
        this.y_labels = $(this.id + ' .graph-y-labels');
        this.bars = $(this.id + ' .graph-bars');
        this.set_labels();
    };
    GraphFeed.prototype = Object.create(Feed.prototype);
    GraphFeed.prototype.set_labels = function ()
    {
        this.y_labels.empty();
        this.bars.empty();
        this.bars.append('<div class="graph-bar-col"><span class="graph-bar-bottom"></span><span class="graph-bar-top"></span></div>');
        this.bars_width = Number($('.graph-bars').css('width').replace('px', ''));
        this.bar_width = Number($('.graph-bar-col').css('width').replace('px', ''));
        this.bar_height = Number($('.graph-bar-col').css('height').replace('px', ''));
        this.bar_count = Math.floor( this.bars_width / this.bar_width );
        if (!isFinite(this.bar_count)) { throw 'invalid graph bar styles; calculated bar count: ' + this.bar_count; }
        for( var i = 1; i < this.bar_count; ++i )
        {
            this.bars.append('<div class="graph-bar-col"><span class="graph-bar-bottom"></span><span class="graph-bar-top"></span></div>');
        }
        var _this = this;
        var got_max = false;
        this.config.graph.thresholds.forEach( function( threshold, index )
        {
            if( threshold.value < _this.config.graph.min || threshold.value > _this.config.graph.max ) { return; }
            if( threshold.value == _this.config.graph.max ) { got_max = true; }
            var span = $('<span class="graph-y-label">' + threshold.value + _this.config.graph.units + '</span>');
            span.css('top', _this.get_bar_height(threshold.value) - 9);
            span.appendTo(_this.y_labels);
        } );
        if( !got_max )
        {
            var span = $('<span class="graph-y-label">' + this.config.graph.max + _this.config.graph.units + '</span>');
            span.css('top', _this.get_bar_height(this.config.graph.max) - 9);
            span.appendTo(this.y_labels);
        }
        {
            var span = $('<span class="graph-y-label">' + this.config.graph.min + _this.config.graph.units + '</span>');
            span.css('top', _this.get_bar_height(this.config.graph.min) - 9);
            span.appendTo(this.y_labels);
        }
        this.bars.find('.graph-bar-col').each(function (i, e)
        {
            _this.config.graph.thresholds.forEach(function (threshold, index)
            {
                if (threshold.value > _this.config.graph.max) { return; }
                var span = $('<span class="graph-threshold"></span>');
                span.css('height', _this.get_bar_height(threshold.value) + 1 + 'px');
                span.css('border-bottom-color', threshold.color);
                span.appendTo(e);
            });
        });
        $('.graph-bar-col').tooltip();
    }
    GraphFeed.prototype.load = function ()
    {
        $.ajax({
            context: this,
            crossDomain: true,
            url: this.get_url(),
            timeout: globals.timeout
        }).done(function (data, textStatus, jqXHR) {
            this.onload(data);
        }).fail(function (jqXHR, textStatus, errorThrown) {
            this.onerror();
        });
    };
    GraphFeed.prototype.onload_ = function( data )
    {
        var strings = [];
        var text = 'n/a'
        if( data )
        { 
            data.replace( "\n", '' );
            strings = data.split( ',' );
            if( strings.length == 1 )
            { 
                text = this.config.graph.units ? data + ' ' + this.config.graph.units : data;
            }
            else
            {
                for( var i in strings ) { text = strings[i] + ' ';  } // todo!
            }
        }
        this.text.html( text );
        var default_value = this.config.graph.min < 0 && this.config.graph.max > 0 ? 0 : this.config.graph.max <= 0 ? this.config.graph.max : this.config.graph.min;
        values = []
        for( var i in strings ) { values.push( isNaN( strings[i] ) ? default_value : Number( strings[i] ) ); }
        var bar = this.bars.children().first();
        this.bars.append( bar );
        var bottom_height = this.get_bar_bottom( values[0] );
        var top_height = this.get_bar_top( values[0] );
        var threshold = this.get_threshold( values[0] );
        bar.find('.graph-bar-bottom').css('height', bottom_height).css('background', threshold.color);
        bar.find('.graph-bar-top').css('height', top_height);
        if( this.config.alert ) { this.alert( threshold.alert ); }
        bar.data('bs.tooltip').options.title = text + ' @ ' + this.refresh_time;
        bar.find('.graph-threshold').each( function( index, value )
        {
            var e = $( value );
            var height = Number( e.css( 'height' ).replace( 'px', '' ) );
            if( height > top_height && height < bottom_height ) { e.hide(); } else { e.show(); }
        });
    };
    GraphFeed.prototype.get_bar_height = function( value )
    {
        var scale = ( value - this.config.graph.min ) / ( this.config.graph.max - this.config.graph.min );
        scale = scale > 1 ? 1 : scale < 0 ? 0 : scale;
        return this.bar_height - scale * this.bar_height;
    };
    GraphFeed.prototype.get_bar_top = function( value )
    {
        if (this.config.graph.max > 0 && this.config.graph.min < 0) { return this.get_bar_height( value < 0 ? 0 : value ); }
        if (this.config.graph.max <= 0) { return this.get_bar_height( this.config.graph.max ); }
        return this.get_bar_height( value );
    };
    GraphFeed.prototype.get_bar_bottom = function( value )
    {
        if (this.config.graph.max > 0 && this.config.graph.min < 0) { return this.get_bar_height( value >= 0 ? 0 : value ); }
        if (this.config.graph.min >= 0) { return this.get_bar_height( this.config.graph.min ); }
        return this.get_bar_height( value );
    };
    GraphFeed.prototype.get_threshold = function ( value )
    {
        if( !this.config.graph.thresholds.length ) { return this.default_threshold; }
        for( var i in this.config.graph.thresholds ) { if ( value <= this.config.graph.thresholds[i].value ) { return this.config.graph.thresholds[i]; } }
        return this.default_exceeded_threshold;
    };
    return GraphFeed;
});
