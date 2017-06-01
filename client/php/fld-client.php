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
 *  the "face morphing" task to the given images (source and destination)
 *  in the command-line shell using PHP script.
 * 
 * Usage: 
 *  /usr/bin/php fld-client.php [img1-source] [img2-destination]
 *
 */

error_reporting(E_ALL);

/** Constants */
$FLDM_SERVER_PORT   = "9876";
$FLDM_SERVER_IP     = "127.0.0.1";

if(sizeof($argv) < 3){
	echo "Usage: /usr/bin/php fld-client.php [img1-source] [img2-destination]\n\n";
	exit;
}

/** set an array for the image files from the arguments */
$files = array($argv[1], $argv[2]);

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
 * - 4 bytes: total file size
 * - (4 + file size) x (# of files) bytes 
 */
socket_write($socket, pack("L", 4 + 4*sizeof($arr_filesize) + $total_filesize));
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
$data_per_loop  = 2048;
$data_received  = 0;
do{
    $data .= socket_read($socket, $data_per_loop);
    $data_received += $data_per_loop;
}while($data_received < $data_len);

echo "data received from the FLDM server.\n";
echo "data_len: ".strlen($data)."\n";
echo "data: ".$data."\n\n";

socket_close($socket);
echo "socket closed.\n";
?>
