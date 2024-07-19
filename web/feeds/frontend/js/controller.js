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

//---------------------------------------------//
//
// Global Constants
//
// --------------------------------------------//

//---------------------------------------------//
//
// Global Utils
//
// --------------------------------------------//
//-----------------------------//
// util
// -----------------------------//

//-----------------------------//
// model
// -----------------------------//
// var feed;


// var feeds = {};
// var pending = {};
// var config_files = [];
// var gui;
//var audio_context = audioContextCheck();
require(['jquery', "jquery_ui",
    "dat_gui", 'base_controller'], function ($) {

    var Feed = require('Feed');
    var ImageFeed = require('ImageFeed');
    var TextFeed = require('TextFeed');
    var GraphFeed = require('GraphFeed');
    var ImageStreamFeed = require('ImageStreamFeed');
    var CsvFeed = require('CsvFeed');
    var TrackFeed = require('TrackFeed');
    // var TrackOptions = require('TrackOptions');
    var MapFeed = require('MapFeed');
    // var MapOptions = require('MapOptions');
    // var GridOptions = require('GridOptions');
    var StartStopFeed = require('StartStopFeed');
    var base_controller = require('base_controller');

    var controller = function (container_width) {
        this.base = base_controller;
        this.base(container_width, false);
        // this.init();
    };
    controller.prototype = Object.create(base_controller.prototype);

    var load_config = function (file) {
        $.ajax({
            url: controller.prototype.config_url(file)
        }).done(function (data, textStatus, jqXHR) {
            initialize(data);
        }).fail(function (jqXHR, textStatus, errorThrown) {
            $('#container').append('<p>error reading <a href="' + file + '">' + file + '</a>: ' + textStatus + ' ' + errorThrown + '</p>');
            $('#container').append('<p>see: <a href="readme.txt">readme.txt</a></p>');
            $('#container').append('<p>see: <a href="examples/web.frontend.json">examples/web.frontend.json</a></p>');
        });
    };
    var add_panel = function (feed_name, feed_path) {
        var class_str = globals.isMobile ? '' : ' transparent'; // todo? make it configurable in frontend.json
        $('#container').append(
            '<li id="' + feed_name + '" class="panel" data-name="' + feed_path + '" >' +
            '  <button type="button" class="panel-close hideable text-muted pull-right' + class_str + '" title="close"><span>&times;</span></button>' +
            '</li>'
        );
    };

    var create_feed = function (type, feed_name, feed_path, config) {
        add_panel(feed_name, feed_path);
        if (type == 'image') {
            var element_str = '<div class="target"/>';
            controller.prototype.add_poll_body(feed_path, element_str, config.form);
            return new ImageFeed(feed_name, feed_path, config);
        } else if (type == 'text' || type == 'csv' || type == 'csv-table') {
            controller.prototype.add_poll_body(feed_path, '<table class="target"><thead></thead></table>', config.form);
            return type == 'text' ? new TextFeed(feed_name, feed_path, config) : new CsvFeed(feed_name, feed_path, config);
        } else if (type == 'graph') {
            controller.prototype.add_poll_body(feed_path, '<div class="target graph"><div class="graph-text">&nbsp;</div><div class="graph-y-labels"></div><div class="graph-bars"></div></div>', config.form);
            return new GraphFeed(feed_name, feed_path, config);
        } else if (type == 'stream') {
            controller.prototype.add_stream_body(feed_path, '<div class="target stream"/>', config.form);
            return new ImageStreamFeed(feed_name, feed_path, config);
        } else if (type == 'track') {
            controller.prototype.add_poll_body(feed_path, '<div class="target"/>', config.form);
            return new TrackFeed(feed_name, feed_path, config);
        } else if (type == 'map') {
            controller.prototype.add_poll_body(feed_path, '<div class="target map"></div>', config.form);
            return new MapFeed(feed_name, feed_path, config);
        } else if (type == 'start_stop') {
            controller.prototype.add_poll_body(feed_path, '<div class="target form"></div>');
            return new StartStopFeed(feed_name, feed_path, config);
        }

        throw 'unrecognised feed type: ' + type;
    };

    $(function () {
        container_width = $('#container').width();
        controller.prototype.init_load_config(load_config, create_feed);
    });


    function initialize(frontend_config) {
        controller.prototype.initialize_gui(500);
        controller.prototype.add_gui_globals();
        controller.prototype.load_feed_items(frontend_config, frontend_config.feeds, "");
        controller.prototype.initialize_load_config();
        controller.prototype.init_actions();
    }


    // function load_config(file) {
    //     $.ajax({
    //         url: file
    //     }).done(function (data, textStatus, jqXHR) {
    //         initialize(data);
    //     }).fail(function (jqXHR, textStatus, errorThrown) {
    //         $('#container').append('<p>error reading <a href="' + file + '">' + file + '</a>: ' + textStatus + ' ' + errorThrown + '</p>');
    //         $('#container').append('<p>see: <a href="readme.txt">readme.txt</a></p>');
    //         $('#container').append('<p>see: <a href="examples/web.frontend.json">examples/web.frontend.json</a></p>');
    //     });
    // }

    // var query_string = parse_query_string();


    // var create_feed = function (type, feed_name, feed_path, config) {
    //     add_panel(feed_name, feed_path);
    //     if (type == 'image') {
    //         var element_str = '<div class="target"/>';
    //         add_poll_body(feed_path, element_str);
    //         return new ImageFeed(feed_name, feed_path, config);
    //     } else if (type == 'text' || type == 'csv' || type == 'csv-table') {
    //         add_poll_body(feed_path, '<table class="target"><thead></thead></table>');
    //         return type == 'text' ? new TextFeed(feed_name, feed_path, config) : new csv_feed(feed_name, feed_path, config);
    //     } else if (type == 'graph') {
    //         add_poll_body(feed_path, '<div class="target graph"><div class="graph-text">&nbsp;</div><div class="graph-y-labels"></div><div class="graph-bars"></div></div>');
    //         return new GraphFeed(feed_name, feed_path, config);
    //     } else if (type == 'stream') {
    //         add_stream_body(feed_path, '<div class="target stream"/>');
    //         return new ImageStreamFeed(feed_name, config);
    //     } else if (type == 'track') {
    //         add_poll_body(feed_path, '<div class="target"/>');
    //         return new TrackFeed(feed_name, feed_path, config);
    //     } else if (type == 'map') {
    //         add_poll_body(feed_path, '<div class="target map"></div>');
    //         return new MapFeed(feed_name, feed_path, config);
    //     } else if (type == 'form') {
    //         add_poll_body(feed_path, '<div class="target form"></div>');
    //         return new FormFeed(feed_name, feed_path, config);
    //     }
    //     throw 'unrecognised feed type: ' + type;
    // };

});

