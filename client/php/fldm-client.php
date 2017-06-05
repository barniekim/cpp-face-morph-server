<?php
/**
 * ======================================
 * FLDM Client (PHP)  
 * Barnabas Kim <his.barnabas@gmail.com>
 * ======================================
 *
 * Description:
 *  A FLDM (Face Landmark Detection & Morphing) client script,
 *  which queries TCP-request to the FLDM server which performs 
 *  the "face morphing" task to the given images (source and target)
 *  in the command-line shell using PHP script.
 * 
 * Usage: 
 *  /usr/bin/php fld-client.php [img1-source] [img2-target] [request-type] [target-result-idx] 
 * 
 *  [img1-source] string; a path of the source image
 *  [img2-target] string; a path of the target image
 *  [request-type] string; composable, progressive
 *  [target-result-idx] integer(1..9); an index of the morphed result out of 10 images. Actually this represents how much 
 *                                     the source or target image to be applied to the result. 
 *                                     For example, if target-result-idx = 5, the result (morphed) will be composed 
 *                                     with 50%  of source as well as 50%. of target image.
 */

error_reporting(E_ALL);

/** constants */
$FLDM_SERVER_PORT           = "9876";
$FLDM_SERVER_IP             = "127.0.0.1";
$REQUEST_TYPE_COMPOSABLE    = 0;
$REQUEST_TYPE_PROGRESSIVE   = 10;

if(sizeof($argv) < 5){
	echo "Usage: /usr/bin/php fld-client.php [img1-source] [img2-target] [request-type] [target-result-idx]\n\n";
	exit;
}

/**
 * write a JPEG-image from the given base64 string
 */
function base64_to_jpeg($str_base64, $filename){
    $fp = fopen($filename, 'wb'); 
    fwrite($fp, base64_decode($str_base64));
    fclose($fp); 
}

/** set an array for the image files from the arguments */
$files = array($argv[1], $argv[2]);

/** set request-type */
$request_type = ($argv[3] == "composable") ? $REQUEST_TYPE_COMPOSABLE : $REQUEST_TYPE_PROGRESSIVE;

/** set target-result-idx */
$target_result_idx = $argv[4];
if(!$target_result_idx){
    $target_result_idx = 5; // 50%:50% as default
}

/** create a TCP/IP socket. */
$socket = socket_create(AF_INET, SOCK_STREAM, SOL_TCP);
if($socket === false){
    fwrite(STDERR, "failed to create a socket: ".socket_strerror(socket_last_error())."\n");
    die();
}
echo "a socket created\n";

/** connect to the socket */
$result = socket_connect($socket, $FLDM_SERVER_IP, $FLDM_SERVER_PORT);
if($result === false){
    fwrite(STDERR, "failed to connect to the socket: ($result)".socket_strerror(socket_last_error($socket))."\n");
    die();
}
echo "connected to the socket.\n";

/** open & read binary data from each file */
$total_filesize = 0;
$arr_contents = array();
foreach($files as $filename){
	$filesize = filesize($filename);
	$fp = fopen($filename, "rb");
	$contents = fread($fp, $filesize);
	fclose($fp);

	$total_filesize += $filesize;
	$arr_contents[] = $contents;
	$arr_filesize[] = $filesize;
}
echo "files have been read.\n";

/** 
 * write socket data:
 * 1) 4 bytes: total file size
 * 2) 4 bytes: request_type (int)
 *    0: Composable (REQUEST_TYPE_COMPOSABLE)
 *    1: Progressive (REQUEST_TYPE_PROGRESSIVE)
 * 3) 4 bytes: target_result_index
 * 4) # of files
 * 5) (4 + file size) x (# of files) bytes 
 */
socket_write($socket, pack("L", 4 + 4*sizeof($arr_filesize) + $total_filesize));
socket_write($socket, pack("L", $request_type));
socket_write($socket, pack("L", $target_result_idx));
socket_write($socket, pack("L", sizeof($arr_filesize)));
for($i=0; $i<sizeof($arr_contents); $i++){
	$contents = $arr_contents[$i];
	$filesize = $arr_filesize[$i];
	socket_write($socket, pack("L", $filesize));
	socket_write($socket, $contents);
}
echo "the socket has been written with the binary data.\n";

/** get response from the server */
$data           = "";
$data_len       = unpack("I", socket_read($socket, 4))[1];
$data_per_loop  = 1;
$data_received  = 0;
do{
    $data .= socket_read($socket, $data_per_loop);
    $data_received += $data_per_loop;
}while($data_received < $data_len);

echo "data received ".strlen($data)." from the FLDM server.\n";

/** decode JSON-string to the JSON-object */
$jsn = json_decode(trim($data));

/** write the results into the JPEG-image file */
$files = array(
    "src" => $jsn->{"encoded_src"},
    "dst" => $jsn->{"encoded_dst"},
    "out" => $jsn->{"encoded_out"}
);
foreach($files as $key=>$str_base64){
    base64_to_jpeg($str_base64, "out/".$key.".jpeg");
}

/** close the socket */
socket_close($socket);
echo "socket closed.\n";
?>
