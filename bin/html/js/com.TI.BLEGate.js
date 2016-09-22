/**
*  TI BLE Device Gateway
*  Allows us to pull data from a board and push values into a display grid.
*
* Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
* 
* Permission is hereby granted, free of charge, to any person obtaining
* a copy of this software and associated documentation files (the
* "Software"), to deal in the Software without restriction, including
* without limitation the rights to use, copy, modify, merge, publish,
* distribute, sublicense, and/or sell copies of the Software, and to
* permit persons to whom the Software is furnished to do so, subject to
* the following conditions:
*  
* The above copyright notice and this permission notice shall be
* included in all copies or substantial portions of the Software.
*  
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
* LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
* OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
* WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
$(function () {
	//var postData = $(this).serializeArray();
	//var sParam = "__SL_P_PS1";
	//postData.push({ name: 'token', value: sParam });
    // go get the tokens for the scanned devices profiles
	
	var postData = $('#scanForm').serialize();
    $.ajax({
        "type": "GET",
        "url": "scan_config.html",
        "cache": false,
		"async": false,
		"data": postData,
        "dataType": "html"
    })
    // define callback for get request success
    .done(function (getdata, status, xhr) {
		var sParam = "__SL_P_PS2";
		var datapage = $(getdata),
			numDevices = 5;

		scannedTable(datapage, sParam, numDevices);
    })
    .fail(function (jqxhr, settings, exception) {
        var sTemp = exception;
    });	

	
	var getData = $(this).serializeArray();
	var sParam = "__SL" + "_G_NSC";
	var sValue = $("#dev_scan > tbody > tr").length;
	getData.push({ name: sParam, value: sValue });
	
	sParam = "__SL" + "_G_NCO";
	sValue = $("#dev_conn > tbody > tr").length;
	getData.push({ name: sParam, value: sValue });	
	
    $.ajax({
        "type": "GET",
        "url": "scan_config.html",
        "cache": false,
		"data": getData,
        "dataType": "html"
    })
    // define callback for get request success
    .done(function (getdata, status, xhr) {
        var sParam = "";
        var datapage = $(getdata),
            numDevices = 3;

		sParam = "__SL_P_PS4";
		connectedTable(datapage, sParam, numDevices);
    })
    .fail(function (jqxhr, settings, exception) {
        var sTemp = exception;
    });
	
    $('#scanForm').on('submit', function (event) {
        event.preventDefault();
        // attach a message for the user
        var $form = $(this),
            submitBtn = $form.find(':submit'),
            statusMessage = submitBtn.parent().find('.status'),
            bClearForm = (submitBtn.val().indexOf('Add') > -1);
        if (statusMessage.length == 0) {
            statusMessage = $('<p class="status">Applying changes...</p>');
            submitBtn.parent().append(statusMessage);
        } else {
            statusMessage.show();
        }

        // do the post to 'action' defined by form html
        // serialize the form data for submission
        var postData = $(this).serializeArray();
        var sParam = "__SL_P_PS1";
        postData.push({ name: 'token', value: sParam });	
		
        $.post($form.attr('action'), $form.serialize())
            // define callback function for when the submission completes
            .done(function (data, textStatus, jqXHR) {
                // textStatus returns 'success' if http status 200 or similar
                if (textStatus == 'success') {
                    statusMessage.html('Changes applied.').delay(3000).fadeOut(400);
                    if (bClearForm) {
                        $form.each(function () {
                            this.reset();
                        });
                    }
                    location.reload();
                } else {
                    statusMessage.html('Failed to apply changes.').delay(3000).fadeOut(400);
                }
            })
            // define callback function for if the form fails
            .fail(function (jqxhr, settings, exception) {
                statusMessage.html('Failed to apply changes.').delay(3000).fadeOut(400);
            });

    });

    $('#scanned_form').on('submit', function(event){
        event.preventDefault();
		
		if($("#scanned_devices > tr > td > input:checked" ).length > 0) {
			// attach a message for the user
			var $form = $(this),
				submitBtn = $form.find(':submit'),
				statusMessage = submitBtn.parent().find('.status'),
				bClearForm = (submitBtn.val().indexOf('Add') > -1);
			if (statusMessage.length == 0) {
				statusMessage = $('<p class="status">Applying changes...</p>');
				submitBtn.parent().append(statusMessage);    
			} else {
				statusMessage.show();
			}
			var postData = $(this).serializeArray();
			var sParam = "__SL_P_PS3";
			postData.push({ name: 'token', value: sParam });		
			// do the post to 'action' defined by form html
			// serialize the form data for submission

			$.post($form.attr('action'), postData)
				// define callback function for when the submission completes
				.done(function(data, textStatus, jqXHR) {
					// textStatus returns 'success' if http status 200 or similar
					if (textStatus == 'success') {
						statusMessage.html('Changes applied.').delay( 3000 ).fadeOut( 400 );
						if (bClearForm) {
							$form.each(function(){
								this.reset();                            
							});
						}
						location.reload();
					} else {
						statusMessage.html('Failed to apply changes.').delay( 3000 ).fadeOut( 400 );
					}
				})
				// define callback function for if the form fails
				.fail(function(jqxhr, settings, exception) {
					statusMessage.html('Failed to apply changes.').delay( 3000 ).fadeOut( 400 );
				});
		}
    }); 
    
	$('#connected_form').on('submit', function(event){
        event.preventDefault();
		if($("#dev_conn input:checked" ).length > 0) {
			// attach a message for the user
			var $form = $(this),
				submitBtn = $form.find(':submit'),
				statusMessage = submitBtn.parent().find('.status'),
				bClearForm = (submitBtn.val().indexOf('Add') > -1);
			if (statusMessage.length == 0) {
				statusMessage = $('<p class="status">Applying changes...</p>');
				submitBtn.parent().append(statusMessage);    
			} else {
				statusMessage.show();
			}
			
			var postData = $(this).serializeArray();
			var sParam = "__SL_P_PS5";
			postData.push({ name: 'token', value: sParam });
			// do the post to 'action' defined by form html
			// serialize the form data for submission

			$.post($form.attr('action'), postData)
				// define callback function for when the submission completes
				.done(function(data, textStatus, jqXHR) {
					// textStatus returns 'success' if http status 200 or similar
					if (textStatus == 'success') {
						statusMessage.html('Changes applied.').delay( 3000 ).fadeOut( 400 );
						if (bClearForm) {
							$form.each(function(){
								this.reset();                            
							});
						}
						location.reload();
					} else {
						statusMessage.html('Failed to apply changes.').delay( 3000 ).fadeOut( 400 );
					}
				})
				// define callback function for if the form fails
				.fail(function(jqxhr, settings, exception) {
					statusMessage.html('Failed to apply changes.').delay( 3000 ).fadeOut( 400 );
				});
		}
    }); 	
	
    $('.but_conn_refresh').click(function (event) {
        event.preventDefault();
        // attach a message for the user
        var postData = $(this).serializeArray();
        var sParam = "__SL" + "_G_NSC";
		var sValue = $("#dev_scan > tbody > tr").length;
        postData.push({ name: sParam, value: sValue });
        
		sParam = "__SL" + "_G_NCO";
		sValue = $("#dev_conn > tbody > tr").length;
		postData.push({ name: sParam, value: sValue });

		$.ajax({
			"type": "GET",
			"url": "scan_config.html",
			"data": postData,
			"cache": false,
			"dataType": "html"
		})
		// define callback for get request success
		.done(function (getdata, status, xhr) {
			var sParam = "__SL" + "_P_PS2";
			var datapage = $(getdata),
			numDevices = 5;
			scannedTable(datapage, sParam, numDevices);
			
			sParam = "__SL" + "_P_PS4";
			numDevices = 3
			connectedTable(datapage, sParam, numDevices)
			
		})
		.fail(function (jqxhr, settings, exception) {
			var sTemp = exception;
		});
    });
	
	$('#ble_devices').on('click', 'input', function (event) {
		var $form = $(this);
        $.post('#', $form.serialize())
            // define callback function for when the submission completes
            .done(function (data, textStatus, jqXHR) {

            })
            // define callback function for if the form fails
            .fail(function (jqxhr, settings, exception) {
                statusMessage.html('Failed to apply changes.').delay(3000).fadeOut(400);
            });		
	});
	
	$('#bleConnectedForm').on('submit', function(event){
        event.preventDefault();
		if($("#ble_devices input:checked" ).length > 0) {
			// attach a message for the user
			var $form = $(this);
			
			// do the post to 'action' defined by form html
			// serialize the form data for submission				
			$.ajax({
				"type": "POST",
				"url": "ble_config.html",
				"cache": false,
				"async": false,
				"data": $form.serialize(),
				"dataType": "html"
			})
			// define callback for get request success
			.done(function (getdata, status, xhr) {
						var datapage = $(getdata);
						PopulateCharacteristicTable(datapage);			
			})
			.fail(function (jqxhr, settings, exception) {
				var sTemp = exception;
			});				
		}		
	});

	$('#devCharacteristicsForm').on('submit', function(event){
        event.preventDefault();
		if($("#devCharacteristicsForm input:checked" ).length > 0) {
			var iVal = 0;
			if( $("#chkSet").attr('checked') == 'checked' ) {
				iVal = 1;
			}
			// attach a message for the user
			var $form = $(this);
			
			var postData = $(this).serializeArray();

			postData.push({ name: "__SL_P_PS8", value: $('#ble_devices input:checked').val() });				
			if( $("#chkSet").attr('checked') == 'checked' ) {
				postData.push({ name: "__SL_P_P10", value: $("#setVal").val() });			
			}
			postData.push({ name: "__SL_P_P11", value: iVal });
			
			// do the post to 'action' defined by form html
			// serialize the form data for submission				
			$.ajax({
				"type": "POST",
				"url": "ble_config.html",
				"cache": false,
				"data": postData,
				"async": false,
				"dataType": "html"
			})
			// define callback for get request success
			.done(function (getdata, status, xhr) {
				setTimeout(function() {
					getBLEResponse(getdata, iVal);
				}, 2000);				 
			})
			.fail(function (jqxhr, settings, exception) {		
			});				
		}		
	});

	
});

function getBLEResponse(getdata, iVal) {
	var datapage = $(getdata);	
	var retVal;
	var getMsg = "Value of the characteristic&nbsp;=&nbsp;";
	
	if(iVal == 0) {
		retVal = datapage.filter('#DeviceGetValue').text();
		$('#lblGetMsg').show();		
		$('#lblSetMsg').attr('style','color:Green;')
		$('#lblGetMsg').html(getMsg+retVal);	
	}
	else { //lblSetMsg
		retVal = datapage.filter('#DeviceSetValue').text();
		$('#lblSetMsg').show();		
		$('#lblSetMsg').attr('style','color:#cc0000;')
		$('#lblSetMsg').html(retVal);		
	}
}

function PopulateCharacteristicTable(datapage) {
    var buttonDisp = false;
    var prop1;
    var prop2;
    var prop3;

    var checkpos = -1;

    var j = 0;

	//numDevices = datapage.filter('NumberOfScanned').text();
	$('#characteristics').html('');
	for (i = 0; i < 150; i++) {


		if ( (i%3 == 0) && (i > 0) ) {
			buttonDisp = true;

			if( prop3.toUpperCase() != 'XXXXXX') {
				$("#characteristics").append('<tr data-devid="' + prop1 + '"> \
					<td><input type="radio" name="__SL_P_PS9" value="' + prop3 + '">' + prop1 + '</td>\
					<td>' + prop2 + '</td>\
					<td>' + prop3 + '</tr>');			
			}				
		}
		if( i % 3 == 0) {
			prop1 = datapage.filter('#DeviceCharacteristic' + (i + 1)).text();
		}
		if( i % 3 == 1) {
			prop2 = datapage.filter('#DeviceCharacteristic' + (i + 2)).text();
		}
		if( i % 3 == 2) {
			prop3 = datapage.filter('#DeviceCharacteristic' + (i + 3)).text();
		}


		
	}
    if (buttonDisp) {
        $("#but_est_link").show();
    }
    else {
        $("#but_est_link").hide();
    }
    
}

function scannedTable(datapage, sParam, numDevices) {
    var buttonDisp = false;
    var deviceID;
    var devName;
    var devBDAddress;
    var devAddrType;
    var checkpos = -1;

    var j = 0;

	//numDevices = datapage.filter('NumberOfScanned').text();
	$('#scanned_devices').html('');
	for (i = 0; i < numDevices; i++) {

		deviceID = datapage.filter('#ScannedDeviceUA' + (i + 1)).text();
		devName = datapage.filter('#ScannedDeviceUB' + (i + 1)).text();
		devBDAddress = datapage.filter('#ScannedDeviceUC' + (i + 1)).text();
		devAddrType = datapage.filter('#ScannedDeviceUD' + (i + 1)).text();

		if ((deviceID.toUpperCase() != 'X') && (devName.toUpperCase() != 'XXXXXX')) {
			buttonDisp = true;
			$("#scanned_devices").append('<tr data-devid="' + deviceID + '"> \
				<td><input type="radio" name="' + sParam + '" value="' + deviceID + '">' + deviceID + '</td>\
				<td>' + devName + '</td>\
				<td>' + devBDAddress + '</td>\
				<td>' + devAddrType + '</td></tr>');				
		}
		
	}
    if (buttonDisp) {
        $("#but_est_link").show();
    }
    else {
        $("#but_est_link").hide();
    }
    
}

function connectedTable(datapage, sParam, numDevices) {
    var buttonDisp = false;
    var deviceID;
    var devName;
    var devBDAddress;
    var devAddrType;

    var checkpos = -1;

    var j = 0;

	//numDevices = datapage.filter('NumberOfConnected').text();
	$('#connected_devices').html('');
	$('#ble_devices').html('');
	for (i = 0; i < numDevices; i++) {

		deviceID = datapage.filter('#ConnectedDeviceDA' + (i + 1)).text();
		devName = datapage.filter('#ConnectedDeviceDB' + (i + 1)).text();
		devBDAddress = datapage.filter('#ConnectedDeviceDC' + (i + 1)).text();
		devAddrType = datapage.filter('#ConnectedDeviceDD' + (i + 1)).text();

		if ((deviceID.toUpperCase() != 'X') && (devName.toUpperCase() != 'XXXXXX')) {
			buttonDisp = true;
			$("#connected_devices").append('<tr data-devid="' + deviceID + '"> \
				<td><input type="radio" name="' + sParam + '" value="' + deviceID + '">' + deviceID + '</td>\
				<td>' + devName + '</td>\
				<td>' + devBDAddress + '</td>\
				<td>' + devAddrType + '</td></tr>');
			// add to BLE tab
			$("#ble_devices").append('<tr data-devid="' + deviceID + '"> \
				<td><input type="radio" name="__SL_P_PS6" value="' + deviceID + '">' + deviceID + '</input></td>\
				<td>' + devName + '</td>\
				<td>' + devBDAddress + '</td>\
				<td>' + devAddrType + '</td></tr>');				
		}
		
	}
    if (buttonDisp) {
        $("#but_terminate").show();
    }
    else {
        $("#but_terminate").hide();
    }

    
}