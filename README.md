# drone-project

드론 프로젝트

## 드론 정보
 
>    후방, 좌측 모터 9번핀    
>    후방, 우측 모터 10번핀    
>    전방, 좌측 모터 3번핀    
>    전방, 우측 모터 11번핀  
>    sonar 3개 (측면 2개, 하방 1개)
>    raider 2개

### 파일

> motor_sensor_variables.h  

모터와 센서 변수들이 선언되어 있다.  

> main.cpp  

전체적인 코드  

> gyro_variables.h  

자이로 센서와 관련된 변수들이 선언 되어있다.

> function_proto.h  

함수들의 프로토 타입이 선언되어 있다.

> header.h  

헤더파일들이 선언되어있다.


#### 참고할 링크들

 http://blog.naver.com/PostView.nhn?blogId=rlrkcka&logNo=221380249135&parentCategoryNo=&categoryNo=18&viewDate=&isShowPopularPosts=false&from=postView  
 http://reefwingrobotics.blogspot.com/2018/04/arduino-self-levelling-drone-part-5.html  
 https://sensibilityit.tistory.com/455?category=657462  
 http://kr.bluesink.io/t/mpu-9250/36/5    
 https://raduino.tistory.com/13  
 https://oscarliang.com/quadcopter-pid-explained-tuning/
 
 ##### 기본 규칙
 
센서는 앞에 s가 붙음  
모터는 앞에 m이 붙음  
함수는 구분자가 대문자  
변수는 구분자가 언더바  
항상 왼쪽을 true  
오른쪽을 false라고 한다  


###### 단위

초 : mm
거리 : cm