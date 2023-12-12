import React,{useRef, useEffect, useState} from 'react';
import './Bhambin.css';


const Bhambin = () => {

    const [isShowWindow, setShowWindow] = useState(false);

    function ShowWindow() {
      setShowWindow(true)
    };

    function CloseWindow() {
      setShowWindow(false)
    }

    //status info 
    const [Battery, SetBattery] = useState('-');
    const [Power, SetPower] = useState('-');
    const [WetAmount, SetWetAmount] = useState('-');
    const [GenAmount, SetGenAmount] = useState('-');
    const [RecAmount, SetRecAmount] = useState('-');
    const [WetWeight, SetWetWeight] = useState('-');
    const [GenWeight, SetGenWeight] = useState('-');
    const [RecWeight, SetRecWeight] = useState('-');

    const [WetPic, SetWetPic] = useState('../icon_iot/70wet_0.png');
    const [GenPic, SetGenPic] = useState('../icon_iot/63general_0.png');
    const [RecPic, SetRecPic] = useState('../icon_iot/46recycle_0.png');


    const GetInfoFromServer = async () => {
      try {
        const response_info = await fetch('http://127.0.0.1:5000/get_info_status');
        const infostatus = await response_info.json();

        SetBattery('-');
        SetPower(infostatus['power']);
        SetWetAmount(infostatus['wet_amount']);
        SetGenAmount(infostatus['gen_amount']);
        SetRecAmount(infostatus['rec_amount']);
        SetWetWeight('-');
        SetGenWeight('-');
        SetRecWeight(infostatus['rec_weigth']);

      if (infostatus['wet_amount'] >= 25 && infostatus['wet_amount'] <= 50 ) {
        SetWetPic('../icon_iot/69wet_25.png')
      } else if (infostatus['wet_amount'] >= 50 && infostatus['wet_amount'] <= 75 ) {
        SetWetPic('../icon_iot/62wet_50.png')
      } else if  (infostatus['wet_amount'] >= 75 && infostatus['wet_amount'] <= 100 ) {
        SetWetPic('../icon_iot/61wet_75.png')
      } else if  (infostatus['wet_amount'] >= 100) {
        SetWetPic('../icon_iot/68wet_100.png')
      } else {
        SetWetPic('../icon_iot/70wet_0.png')
      }

      if (infostatus['gen_amount'] >= 25 && infostatus['gen_amount'] <= 50  ) {
        SetGenPic('../icon_iot/64general_25.png')
      } else if (infostatus['gen_amount'] >= 50 && infostatus['gen_amount'] <= 75 ) {
        SetGenPic('../icon_iot/65general_50.png')
      } else if  (infostatus['gen_amount'] >= 75 && infostatus['gen_amount'] <= 100 ) {
        SetGenPic('../icon_iot/66general_75.png')
      } else if  (infostatus['gen_amount'] >= 100) {
        SetGenPic('../icon_iot/67general_100.png')
      } else {
        SetGenPic('../icon_iot/63general_0.png')
      }

      if (infostatus['rec_amount'] >= 25  && infostatus['rec_amount'] <= 50 ) {
        SetRecPic('../icon_iot/43recycle_25.png')
      } else if (infostatus['rec_amount'] >= 50  && infostatus['rec_amount'] <= 75 ) {
        SetRecPic('../icon_iot/44recycle_50.png')
      } else if  (infostatus['rec_amount'] >= 75  && infostatus['rec_amount'] <= 100 ) {
        SetRecPic('../icon_iot/45recycle_75.png')
      } else if  (infostatus['rec_amount'] >= 100) {
        SetRecPic('../icon_iot/42recycle_100.png')
      } else {
        SetRecPic('../icon_iot/46recycle_0.png')
      }

      } catch (error) {
        console.error('Error:', error);
      }

    };

    //sent to back
    const CallingYou = async () => {
      // const response_info = await fetch("http://127.0.0.1:5000/callingcar");
      // const carstatus = await response_info.json();
      // cr = carstatus['msg']
      // if (cr == 'waitting') {
        alert('the car is going');
   
      // }
    };

    useEffect(() => {
      const interval = setInterval(() => {
        GetInfoFromServer();
      }, 5000); // Fetch data every 5 seconds (adjust this as needed)
      
      return () => clearInterval(interval); // Clear interval on component unmount
    }, []);

    
    //status bar
    const StatusWindowBar = () => {
      return (
        <div className="trashbar">
              <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/4.7.0/css/font-awesome.min.css"/>
              <button className="closewindow" onClick={CloseWindow}><i className="fa fa-times fa-2x" aria-hidden="true"></i></button>
              <div className='bintitle'>
              <div className='frametextbin'>Wet</div>
              <div className='frametextbin'>General</div>
              <div className='frametextbin'>Recycle</div>
              </div>
              <img className= 'wetbin' src={WetPic}/>
              <img className= 'generalbin' src={GenPic}/>
              <img className= 'rebin' src={RecPic}/>
              <div className='statustext'>
               <div>Battery Level</div>
               <div>Power Status</div>
               <div>Amount</div>
               <div>Weight</div>
              </div>
              <div className='wetstatus'>
                <div className='frametextstatus'>{Battery}</div>
                <div className='frametextstatus'>{Power}</div>
                <div className='frametextstatus'>{WetAmount}</div>
                <div className='frametextstatus'>{WetWeight}</div>
              </div>
              <div className='generalstatus'>
                <div className='frametextstatus'>{Battery}</div>
                <div className='frametextstatus'>{Power}</div>
                <div className='frametextstatus'>{GenAmount}</div>
                <div className='frametextstatus'>{GenWeight}</div>
              </div>
              <div className='restatus'>
                <div className='frametextstatus'>{Battery}</div>
                <div className='frametextstatus'>{Power}</div>
                <div className='frametextstatus'>{RecAmount}</div>
                <div className='frametextstatus'>{RecWeight}</div>
              </div>
              <button className='callcar' onClick={CallingYou}>collect</button>
            </div>
      );
    }

    return (
      <div className="bhambin_page">
        <div className="mainbar">
          <img className= 'showmap' src={'../icon_iot/27map.png'} alt=""/>
          <button className='trash-target' onClick={ShowWindow}> <img className="trashIcon" src={'../icon_iot/26available_bin.png'} alt=""/></button>
          {isShowWindow && StatusWindowBar()}
        </div>

        <div className="icon-bar">
        <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/4.7.0/css/font-awesome.min.css"/>
          <a className="active" href="#"><i className="fa fa-map"></i></a>
          <a href="#"><i className="fa fa-search"></i></a>
        </div>
  
      </div>
    );
  }
  
export default Bhambin;
