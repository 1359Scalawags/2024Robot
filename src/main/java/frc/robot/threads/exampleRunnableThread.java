package frc.robot.threads;

import java.nio.ByteBuffer;
import frc.robot.extensions.RunnableThread;

class testThread implements Runnable{    
    RunnableThread parent;
    ByteBuffer buff;

    testThread(RunnableThread parentRunnableThread){
        parent = parentRunnableThread;
        parent.packBuffer(buff);
    }

    @Override
    public void run(){
        int i = 0;
        while(true){

            if(i%20 == 0){
                ByteBuffer buff = ByteBuffer.allocate(Long.BYTES).putDouble((double)i);
                parent.setBuffer(buff);
            }

            if(i >= 100){
                i = 0;
            }
            i++;
        }
    }
}

public class exampleRunnableThread extends RunnableThread{
    public exampleRunnableThread(){
        setBuffer(ByteBuffer.allocate(Long.BYTES).putDouble(0.001));

        setMethod(new testThread(this));
    }
}