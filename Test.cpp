/*    
 *     
 * Created by Soham 27/04/2020
 *   
 */

#include"InteractiveICP.hpp"

int main(){
    InteractiveICP *n = new InteractiveICP();
    n->file_loader();
    delete n;
}