<?/*************** change only bellow line -> adress to your nightscout site *********/
$Webadress = "https://NIGHTSCOUT_WEBSITE_ADDRESS";



echo "!";
$UnixServerTime = strtotime(date("H:i:s")); //Shows actual time

$jsonData = file_get_contents($Webadress . '/api/v1/entries.json?count=240'); // show last 240 entries
$data = json_decode($jsonData, true);
$tabela = array_fill(0, 240, 0); // Initiate table with 0

/*
foreach ($data as $item) {
    echo "Systime: ". $item["sysTime"];
	echo  " Date: ". $item["date"];
	echo  " BG: ". $item["sgv"];
	echo "<br>";
}*/

//print_r($data);


$jsonData2 = file_get_contents($Webadress . '/api/v1/status.json');
$data2 = json_decode($jsonData2, true);
/*
echo  "<br><br> Nightscout Servertime:". $data2[serverTime];
echo  "<br> Nightscout UNIX Servertime:". $data2[serverTimeEpoch];
*/
//$difftime = abs($UnixServerTime - $data2[serverTimeEpoch]);  not needed?
//echo "difftime is". $difftime;
/////////////////////////////////////
//echo "<br><br><br><br><br><br>";


foreach ($data as $item) {
	$ilesektemu = abs(($item["date"]/1000) - $UnixServerTime );
//	echo "IleMinTemu" . round($ilesektemu/60);
//	echo  " BG: ". $item["sgv"];
//	echo "<br>";
	

	$tabela[floor($ilesektemu/60)]=$item["sgv"];
}

/////////////////
/*
for ($i = 0; $i < 20; $i++) {
   echo "tabela[" . $i . "] = " . $tabela[$i] . "<br>";
}
echo "<br><br><br><br><br><br>";
*/

for ($i = 0; $i < 240; $i++) {
   echo $tabela[$i] . ";";
}
echo "@";

?>