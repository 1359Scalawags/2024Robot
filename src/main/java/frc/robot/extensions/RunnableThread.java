package frc.robot.extensions;

import java.nio.ByteBuffer;

enum ThreadErrorCodes{
    STATUS_OK,
    STATUS_NOTHREAD,
    STATUS_NORUNNABLE,
    STATUS_LOCKED
}


public class RunnableThread{
    Thread thread;
    ByteBuffer byteBuffer;
    Runnable runnableMethod;
    boolean dataLock;

    public RunnableThread(){
        dataLock = false;
    }

    public void setMethod(Runnable method){
        runnableMethod = method;
    }

    void initThread(){
        thread = new Thread(runnableMethod);
    }

    void lock(){
        dataLock = true;
    }

    void unlock(){
        dataLock = false;
    }

    public void waitforUnlock(){
        int counter = 100;
        while(dataLock && counter > 0){counter--;}
    }

    public ThreadErrorCodes putBuffer(ByteBuffer input){
        waitforUnlock();

        if(!dataLock){
            dataLock = true;
            byteBuffer = input.duplicate();
            dataLock = false;
            
            return ThreadErrorCodes.STATUS_OK;
        }

        return ThreadErrorCodes.STATUS_LOCKED;
    }

    public ThreadErrorCodes packBuffer(ByteBuffer result){
        waitforUnlock();

        if(!dataLock){
            dataLock = true;
            result = byteBuffer.duplicate();
            dataLock = false;

            return ThreadErrorCodes.STATUS_OK;
        }

        return ThreadErrorCodes.STATUS_LOCKED;
    }

    public ThreadErrorCodes start(){

        if(runnableMethod == null)
            return ThreadErrorCodes.STATUS_NORUNNABLE;
        else if(thread == null)
            initThread();

        if(thread == null) //if thread is still null then return
            return ThreadErrorCodes.STATUS_NOTHREAD;

        thread.start();
        
        return ThreadErrorCodes.STATUS_OK;
    }
}