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

/*
  Creates a form with <n> text fields and three buttons marked Start, Stop and Clear

  ╭───────────────────────────────╮
  │                               │
  │ field-1   [ value-1 ]         │
  │ field-1   [ value-1 ]         │
  │ ...                           │
  │ ╭───────╮ ╭───────╮ ╭───────╮ │
  │ │ Start │ │ Stop  │ │ Clear │ │
  │ ╰───────╯ ╰───────╯ ╰───────╯ │
  │ [ status ]                    │
  ╰───────────────────────────────╯

  There is an optional Output field that appears above the status line if there
  is an "output" field in the GET response.

  Start and Stop submit the form; Clear clears the text entry fields

  frontend.json:
  "log":
  {
      "type": "start_stop",
      "form":
      {
          "fields":
          {
              <field-1>: <value-1>,
              <field-2>: <value-2>,
              ...
          }
      }
  }

  backend.json:
  "log"
  {
      "command": <handler>
  }

  When Start or Stop is selected, the following request is sent:

  "GET /log?_=<request-id>&command=<start|stop>&<field-1>=<http-escaped-value-1>&<field-2>=<http-escaped-value-2>&... HTTP/1.1"

  The same request is sent on every refresh, without the command field.

  In normal handling this is process by the snark-webframes service which then
  calls the configured handler application with the data passed on stdin as:

  command=<start|stop>
  <field-1>=<value-1>
  <field-2>=<value-2>
  ...

  In response to the GET request the frontend expects to receive a JSON message with:
  {
      "status":
      {
          "code": <0|1>,            // 0 = success
          "message": <string>
      },
      "running":
      {
          "status": <0|1>           // 1 = running
      },
      "fields":
      {
          <field-1>: <value-1>,
          ...
      },
      "output": <string>
  }

  All the fields are optional.

  status.message is placed in the status text field, coloured according to status.code.
  There is a default message of "Success" or "Error" depending on status.code.
  If status.code = 1 (error) the panel is disabled until the error is cleared.

  When running.status = 1 the Start and Clear buttons are disabled; otherwise Stop is disabled.

  Also when running.status = 1, any values set in the fields record are applied
  to the input field of the same name. This allows these fields to be updated to
  reflect some external behaviour (e.g. a value set by the logging program).

  If included, the contents of the "output" field, which can be multi-line is
  placed in a text box above the status field.

 */
define( 'StartStopFeed', ["jquery", "Feed"], function( $ )
{
        var Feed = require('Feed');
        var StartStopFeed = function( feed_name, feed_path, config )
        {
            this.base = Feed;
            this.form_show_buttons = config['form'] == undefined || config.form['fields'] == undefined ? [ "start", "stop" ] : [ "start", "stop", "clear" ];
            this.base( feed_name, feed_path, config );
            this.start_btn = undefined;
            this.stop_btn = undefined;
            // this.buttons = [];
        };
        StartStopFeed.prototype = Object.create( Feed.prototype );
        StartStopFeed.prototype.add_buttons = function( container )
        {
            // To make the "clear" button align consistently, the buttons added here
            // need to occupy 9 columns in total. So if a button isn't shown, we replace
            // it with equivalent padding.
            let pad_cols = 0;
            if( this.button_enabled( "clear" ) ) { var start_size = 4; var stop_size = 3; var gap = 1; } // quick and dirty, for backward compatibility
            else { var start_size = 5; var stop_size = 5; var gap = 2; }
            if( this.button_enabled( "start" ) )
            {
                this.start_btn = add_button(this, "Start", "btn-primary col-sm-" + start_size, "start");
                container.append(this.start_btn).append($('<label/>', {class: "col-sm-" + gap}));
            }
            else
            {
                pad_cols += 5;
            }
            if( this.button_enabled( "stop" ) )
            {
                this.stop_btn = add_button(this, "Stop", "btn-danger col-sm-" + stop_size, "stop");
                container.append(this.stop_btn).append($('<label/>', {class: "col-sm-1"}));
            }
            else
            {
                pad_cols += 4;
            }
            if( pad_cols > 0 && this.button_enabled( "clear" ) )
            {
                container.append($('<label/>', {class: "col-sm-" + pad_cols}));
            }
        };
        // StartStopFeed.prototype.extract_fields = function (form_elements) {
        //     if (form_elements != undefined) {
        //         this.populate_path_values(form_elements, "");
        //     }
        // };
        StartStopFeed.prototype.addListeners = function()
        {
            var this_ = this;
            $($(this.form).find("input")).on("input", function()
            {
                $(this_.form).find("button[name=start]").removeAttr('disabled');
                $(this_.form).find("button[name=clear]").removeAttr('disabled');
            });
            var size = Object.keys(this.fields).length;
            if (size > 0) { this.target.width(this_.width); }
            else { $(this.target).css("min-width", function () { return 200; }); }
            $(this.target).css("min-height", function () { return 100; });
            // var isValidForm = true;
            //
            // var keys = Object.keys(this.fields);
            // for (index in keys) {
            //     var key = keys[index];
            //     if (this.fields[key].length == 0) {
            //         isValidForm = false;
            //     }
            // }
            // if (!isValidForm) {
            //     update_ui_status(0, this.form);
            // }
            // $.ajax({
            //     context: this,
            //     url: this.get_url()
            // }).done(function (data, textStatus, jqXHR) {
            //     this.onload(data);
            // }).fail(function (jqXHR, textStatus, errorThrown) {
            //     this.onerror();
            // });
        };

        // StartStopFeed.prototype.load = function () {
        //     Feed.prototype.load.call(this);
        // };
        StartStopFeed.prototype.load = function()
        {
            $.ajax(
            {
                context: this,
                type: "GET",
                crossDomain: true,
                url: this.get_url(),
                data: $(this.target).find("form").serialize(),
                timeout: globals.timeout
            } )
            .done(function (data, textStatus, jqXHR) { this.onload(data); })
            .fail(function (jqXHR, textStatus, errorThrown) { this.onerror(); });
        };

        StartStopFeed.prototype.update_output = function (data)
        {
            console.log( "==> StartStopFeed.update_output: " + this.feed_name + ": a" );
            // this.target.height(100);
            // data = data.replace(/\n/g, '<br/>');
            this.el.removeClass('panel-disabled');
            // if( this.config.status.show == undefined || this.config.status.show == "false" || this.config.status.show )
            // {
            $(this.form).find(".error-message").remove();
            $(this.form).find(".success-message").remove();
            $(this.form).find(".result-panel").remove();
            // }
            console.log( "==> StartStopFeed.update_output: " + this.feed_name + ": b" );
            if( data.output != undefined )
            {
                var output = data.output;
                var panel = $('<div>', {class: "panel result-panel col-sm-12"});
                panel.append($('<label>', {class: "", text: "Output"}));
                var output_div = $('<div>', {class: "form-results"});
                output_div.append($('<pre>', {text: output}));
                panel.append(output_div); //JSON.stringify(output, null, 4)
                panel.append($('<div>', {class: "clearfix"}));
                $(this.form).append(panel);
                $(this.form).append($('<div>', {class: "clear"}));
                panel = $(this.form).find(".form-results");
                $(panel).scrollTop($(panel)[0].scrollHeight);
            }
            console.log( "==> StartStopFeed.update_output: " + this.feed_name + ": c" );
            if( data.status != undefined )
            {
                var status = data.status;
                if( status.code != undefined )
                {
                    if( status.code == 0 )
                    {
                        $(this.form).append($('<label>', {
                            class: "success-message col-sm-11",
                            text: ( status.message ? status.message : "Success" )
                        }));
                    }
                    else
                    {
                        this.el.addClass('panel-disabled');
                        $(this.form).append($('<label>', {
                            class: "error-message col-sm-11",
                            text: ( status.message ? status.message : "Error" )
                        }));
                    }
                    $(this.form).append($('<div>', {class: "clear"}));
                }
            }
            console.log( "==> StartStopFeed.update_output: " + this.feed_name + ": d" );
        };
        StartStopFeed.prototype.onload_ = function( data )
        {
            if( data == undefined || data == "" ) { return; }
            var json = $.parseJSON(data);
            if( json != undefined && json.status != undefined && json.status.code != undefined && json.status.code == 0 )
            {
                this.update_ui( json );
                this.update_output( json );
            }
        };
        var add_button = function (this_, name, style, command)
        {
            var btn = $('<button/>',
                {
                    value: name,
                    name: command,
                    text: name,
                    class: "btn " + style,
                    click: function (event) {
                        event.preventDefault();
                        var url = this_.get_url();
                        if (url != undefined)
                        {
                            url = url + "&" + $.param({'command': this.name});
                            $.ajax({
                                context: this,
                                type: "GET",
                                crossDomain: true,
                                data: $(this).closest("form").serialize() + "&" + $.param({'command': this.name}),
                                url: url
                            }).done(function (data, textStatus, jqXHR) {
                                var json = $.parseJSON(data);
                                this_.update_output(json);
                                if (json != undefined && json.status != undefined && json.status.code != undefined && json.status.code == 0) {
                                    $(this).attr("disabled", "disabled");
                                    var btn_name = $(this).attr("name");
                                    if( btn_name == "stop" )
                                    {
                                        $(this_.form).find("button[name=start]").removeAttr("disabled");
                                        $(this_.form).find("button[name=clear]").removeAttr("disabled");
                                    }
                                    else
                                    {
                                        $(this_.form).find("button[name=stop]").removeAttr("disabled");
                                        $(this_.form).find("button[name=clear]").attr("disabled", "disabled");
                                    }
                                }
                            }).fail(function (jqXHR, textStatus, errorThrown) {
                                var s;
                                s.status.code = 0;
                                s.status.message = errorThrown;
                                this_.update_output(s);
                                this_.onerror(); // todo: pass json.status.code=0 json.status.message=errorThrown
                            });
                        }
                    }
                });
            return btn;
        };
        var update_ui_status = function( status, form )
        {
            if( status == "1" )
            {
                $(form).find("input").attr("readonly", "readonly"); //[value!='']
                $(form).find("input").attr("title", "readonly"); //[value!='']
                $(form).find("button[name=start]").attr('disabled', "disabled");
                $(form).find("button[name=clear]").attr('disabled', "disabled");
                $(form).find("button[name=stop]").removeAttr('disabled');
            }
            else
            {
                $(form).find("button[name=stop]").attr('disabled', "disabled");
                $(form).find("button[name=clear]").removeAttr('disabled');
                $(form).find("button[name=start]").removeAttr('disabled');
                $(form).find("input").removeAttr("readonly"); //[value!='']
                $(form).find("input").removeAttr("title");
            }
        };
        StartStopFeed.prototype.removeOldStatus = function()
        {
            // $(this.form).find(".error-message").remove();
            // this.el.removeClass('panel-disabled');
        };
        StartStopFeed.prototype.add_form = function () { this.target.append( this.form ); };
        StartStopFeed.prototype.update_ui = function (data)
        {
            if( data.running != undefined && data.running.status != undefined )
            {
                update_ui_status(data.running.status, this.form);
                if (data.running.status == "1")
                {
                    if (data.fields != undefined)
                    {
                        var keys = Object.keys(data.fields);
                        for (index in keys)
                        {
                            var key = keys[index];
                            var input = $(this.form).find("input[name=" + key + "]");
                            input.val(data.fields[key]);
                            // if (input.val().length == 0) {
                            // }
                        }
                    }
                }
            }
        };
        return StartStopFeed;
    }
);
