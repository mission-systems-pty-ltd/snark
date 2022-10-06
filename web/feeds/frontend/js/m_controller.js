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

/**
 * Created by vrushali on 10/04/15.
 *
 *
 * @author    : v.satpute
 * @version    : 1.0
 * @since    : 06/03/2015
 */

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


//---------------------------------------------//
//
// Global Variables
//
// --------------------------------------------//
// var feeds = {};
// var pending = {};
// var config_files = [];
// var gui;
//var audio_context = audioContextCheck();

require(['jquery', 'jquery_mobile', "jquery_timeago", "bootstrap",
    "dat_gui_mobile", 'base_controller'], function ($) {

    globals.config_file = 'config/tab.frontend.json';
    globals.isMobile = true;

    var base_controller = require('base_controller');
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

    var m_controller = function (container_width) {
        this.base = base_controller;
        this.base(container_width, false);
        // this.init();
    };
    m_controller.prototype = Object.create(base_controller.prototype);

    var load_config = function (file) {
        $.ajax({
            url: m_controller.prototype.config_url(file)
        }).done(function (data, textStatus, jqXHR) {
            load_mobile_menu(data);
            initialize(data);
        }).fail(function (jqXHR, textStatus, errorThrown) {
            $('#container').append('<p>error reading <a href="' + file + '">' + file + '</a>: ' + textStatus + ' ' + errorThrown + '</p>');
            $('#container').append('<p>see: <a href="readme.txt">readme.txt</a></p>');
            $('#container').append('<p>see: <a href="examples/web.frontend.json">examples/web.frontend.json</a></p>');
        });
    };

    var create_feed = function (type, feed_name, feed_path, config) {
        add_panel(feed_name, feed_path);
        if (type == 'image') {
            var element_str = '<a href="#popup' + feed_name + '" data-rel="popup" data-position-to="window" data-transition="fade"><div class="target"/></a>';
            var popup_div = '<div data-role="popup" id="popup' + feed_name + '" data-overlay-theme="b" data-theme="b" data-corners="false">' +
                '<a href="#" data-rel="back" class="ui-btn ui-corner-all ui-shadow ui-btn-a ui-icon-delete ui-btn-icon-notext ui-btn-right">Close</a><img class="popphoto" style="max-height:512px;" >' +
                '</div>';
            m_controller.prototype.add_poll_body(feed_path, element_str, config.form, popup_div);
            return new ImageFeed(feed_name, feed_path, config);
        } else if (type == 'text' || type == 'csv' || type == 'csv-table') {
            m_controller.prototype.add_poll_body(feed_path, '<table class="target"><thead></thead></table>', config.form);
            return type == 'text' ? new TextFeed(feed_name, feed_path, config) : new CsvFeed(feed_name, feed_path, config);
        } else if (type == 'graph') {
            m_controller.prototype.add_poll_body(feed_path, '<div class="target graph"><div class="graph-text">&nbsp;</div><div class="graph-y-labels"></div><div class="graph-bars"></div></div>', config.form);
            return new GraphFeed(feed_name, feed_path, config);
        } else if (type == 'stream') {
            m_controller.prototype.add_stream_body(feed_path, '<div class="target stream"/>', config.form);
            return new ImageStreamFeed(feed_name, feed_path, config);
        } else if (type == 'track') {
            m_controller.prototype.add_poll_body(feed_path, '<div class="target"/>', config.form);
            return new TrackFeed(feed_name, feed_path, config);
        } else if (type == 'map') {
            m_controller.prototype.add_poll_body(feed_path, '<div class="target map"></div>', config.form);
            return new MapFeed(feed_name, feed_path, config);
        } else if (type == 'start_stop') {
            m_controller.prototype.add_poll_body(feed_path, '<div class="target form"></div>');
            return new StartStopFeed(feed_name, feed_path, config);
        }
        throw 'unrecognised feed type: ' + type;
    };

    $(function () {
        container_width = $('#container').width();
        m_controller.prototype.init_load_config(load_config, create_feed);
    });


    function initialize(frontend_config) {
        m_controller.prototype.initialize_gui(300);
        m_controller.prototype.add_gui_globals();
        m_controller.prototype.load_feed_items(frontend_config, frontend_config.feeds, "");
        // Global navmenu panel
        $("#main_menu_list").enhanceWithin();
        //$("#main_menu_list").trigger('create');
        $("#main_menu_list").listview();
        m_controller.prototype.init_actions();
        m_controller.prototype.initialize_load_config();
    }

    var add_panel = function (feed_name, feed_path) {
        $('#container').append(
            '<li id="' + feed_name + '" class="panel"  data-name="' + feed_path + '">' +
            '  <button type="button" class="panel-close text-muted pull-right" title="close"><span>&times;</span></button>' +
            '</li>'
        );
    };

    var load_mobile_menu = function (config_data) {
        //var menu_items = [];
        for (var feed_name in config_data.feeds) {
            //menu_items.push(feed_name);
            $("ul#settings").append('<li class="ui-first-child" >' +
                //'<input  class="custom" type="checkbox" name="checkbox-1">' +
                '<a  data="' + feed_name + '" class="">' + feed_name + '</a></li>');
        }

        // Global navmenu panel
        //$(".navmenu-panel ul").listview();


        $("#navmenu-link").on("click", function () {

            $(".main-page").find(".navmenu-panel").panel("open");
        });

        $("#settings a").on('click', function () {
            //alert("clicked" + this.data);
            var elem_id = $(this).attr('data');
            var list_container = $("ul#container");
            //list_container.find('li').addClass("hide");
            //list_container.find('#' + elem_id).removeClass("hide");
        });


    };


});

