static uint8_t CmdDecisionTree()
{
    static uint8_t is_returing_blood = 0;
    static uint8_t is_success_ourpreempt = 0;//我方
    static uint8_t is_success_enemypreempt = 0;//敌方
    static uint8_t RobotstatelevelDesicion(int quantity,int state){    /*先评判状态
    我这边先定机器人数量为chassis_fetch_data.current_robot_quantity*/
    if(quantity == 3 && state > ROBOT_BLOOD_PERFECT){return 1;}
    else if(quantity == 3 && state > ROBOT_BLOOD_HIGH){return 2;}
    else if(quantity == 3){return 3;}
    else if(quantity == 2 && state > ROBOT_BLOOD_HIGH){return 4;}
    else if(quantity == 2){return 5;}
    else if(quantity == 1){return 6;}
    else{return 0;}//表示free state
    }
    static uint8_t is_robot_state_level = RobotstatelevelDesicion(chassis_fetch_data.current_robot_quantity,chassis_fetch_data.blood_warn_state);
    /*机器人自身生存环境评级
    perfect：1
    great：2
    good:3
    fine:4
    just:5
    poor:6*/
    static uint8_t is_ALL_state_level = 0;//手动给即可
    /*敌我双方等级评估
    我方＞敌方：-1
    我方≈敌方：0
    我方<敌方：1*/
    /*决策情况1分析*/
    void PreemptSituation_1(int enemypreempt,int ourpreempt,int blood){
        if(enemypreempt){
            if(blood<ROBOT_BLOOD_DANGER){VisionSetNav(0);}
            if(blood>ROBOT_BLOOD_HIGH){VisionSetNav(2);/*2点巡航也写在上位机*/}
            else if(blood>ROBOT_BLOOD_LOW){VisionSetNav(1);}/*这边1点巡航我打算写在上位机*/}
        else{
            if(blood<ROBOT_BLOOD_DANGER){VisionSetNav(0);}
            if(blood>ROBOT_BLOOD_HIGH){
                if(ourpreempt){VisionSetNav(2);/*辅助游击*/}
                else{VisionSetNav(3);/*尝试占点*/}
            }
            else if(blood>ROBOT_BLOOD_LOW){VisionSetNav(1);}    
        }
    }
    /*决策情况2分析*/
    void PreemptSituation_2(int enemypreempt,int ourpreempt,int blood){
        if(enemypreempt){
            if(blood<ROBOT_BLOOD_LOW){VisionSetNav(0);}
            else if(blood>ROBOT_BLOOD_FULL){VisionSetNav(2);}/*这边2点巡航我打算写在上位机*/}
        else{
            if(blood<ROBOT_BLOOD_LOW){VisionSetNav(0);}
            else if(blood>ROBOT_BLOOD_FULL){
                if(ourpreempt){VisionSetNav(2);/*辅助游击*/}
                else{VisionSetNav(3);/*尝试占点*/}
            }    
        }
    }
    /*决策情况3分析*/
    void PreemptSituation_3(int enemypreempt,int ourpreempt,int blood){
        if(enemypreempt){
            if(blood<ROBOT_BLOOD_LOW){VisionSetNav(0);}
            else if(blood>ROBOT_BLOOD_FULL){VisionSetNav(2);}/*这边2点巡航我打算写在上位机*/}
        else{
            if(blood<ROBOT_BLOOD_LOW){VisionSetNav(0)/*停留机制VisionSetNav(1)*/;}
            else if(blood>ROBOT_BLOOD_FULL){
                if(ourpreempt){VisionSetNav(4);/*激进fire*/}
                else{VisionSetNav(3);/*尝试占点*/}
            }    
        }
    }
    // 停留机制修改
    static time_t stay_start_time = 0;
    static uint8_t stay_state = 1;  // 停留状态

    // 更新停留状态
    static uint8_t UpdateStayState() {
        if (stay_state == 0) {
            return stay_state;  // 如果已经十秒，直接返回
        }
        
        time_t now = time(NULL);
        
        if (stay_start_time == 0) {
            stay_start_time = now;  // 开始计时
        }
        
        if (difftime(now, stay_start_time) >= 10.0) {
            stay_state = 0;  // 停留超时，变为离开状态
        }
        
        return stay_state;
    }

    // 重置为停留状态（回到出生点时调用）
    static void ResetToStayState() {
        stay_start_time = 0;
        stay_state = 1;
    }
    /*兵种打击决策*/
    static int FireDecision(int is_success_enemypreempt){
    }
detect:
    if(chassis_fetch_data.game_process == 4){//比赛中
                    /*战力评估导致的降级系数*/
        is_robot_state_level += is_ALL_state_level;
        if(is_robot_state_level > 6){is_robot_state_level = 6;}
        else if(is_robot_state_level < 1){is_robot_state_level = 1;}

        if(chassis_fetch_data.current_state_left_time <= 60.f){//最后60秒阶段
            switch (is_robot_state_level)
            {
            case 1:
                PreemptSituation_3(is_success_enemypreempt,is_success_ourpreempt,chassis_fetch_data.blood_warn_state);
                break;
            case 2:
                PreemptSituation_3(is_success_enemypreempt,is_success_ourpreempt,chassis_fetch_data.blood_warn_state);
                break;
            case 3:
                PreemptSituation_2(is_success_enemypreempt,is_success_ourpreempt,chassis_fetch_data.blood_warn_state);
                break;
            case 4:
                PreemptSituation_2(is_success_enemypreempt,is_success_ourpreempt,chassis_fetch_data.blood_warn_state);
                break;
            case 5:
                PreemptSituation_1(is_success_enemypreempt,is_success_ourpreempt,chassis_fetch_data.blood_warn_state);
                break;
            case 6:
                PreemptSituation_1(is_success_enemypreempt,is_success_ourpreempt,chassis_fetch_data.blood_warn_state);
                break;
            
            default:
                break;
            }
        }
        else if (chassis_fetch_data.current_state_left_time >60.f)//前期120秒阶段
        {
            /*预留战力评估导致的降级系数*/
             switch (is_robot_state_level)
            {
            case 1:
                PreemptSituation_3(is_success_enemypreempt,is_success_ourpreempt,chassis_fetch_data.blood_warn_state);
                break;
            case 2:
                PreemptSituation_3(is_success_enemypreempt,is_success_ourpreempt,chassis_fetch_data.blood_warn_state);
                break;
            case 3:
                PreemptSituation_2(is_success_enemypreempt,is_success_ourpreempt,chassis_fetch_data.blood_warn_state);
                break;
            case 4:
                PreemptSituation_2(is_success_enemypreempt,is_success_ourpreempt,chassis_fetch_data.blood_warn_state);
                break;
            case 5:
                PreemptSituation_1(is_success_enemypreempt,is_success_ourpreempt,chassis_fetch_data.blood_warn_state);
                break;
            case 6:
                PreemptSituation_1(is_success_enemypreempt,is_success_ourpreempt,chassis_fetch_data.blood_warn_state);
                break;
            
            default:
                break;
            }
        }
        

    }
    /*待定点内容写至上位机，危险距离也写至上位机*/
    return 0;
}
